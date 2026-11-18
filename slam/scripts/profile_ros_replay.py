#!/usr/bin/env python3
"""Reproduce entradas fijas en ROS y captura calidad y tiempos del adaptador.

Sin Gazebo ni RViz; usa un dominio aislado y nunca reproduce velocidades.
La aceleración temporal del player carga el cálculo, no cambia el movimiento
físico que contienen los scans. PYTHONPATH elige la revisión a evaluar.
"""
import argparse
from contextlib import suppress
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import numpy as np
import rclpy
from rclpy.signals import SignalHandlerOptions
from std_srvs.srv import Trigger
import yaml

from run_a_experiment import Observer, bag_info


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--rate', type=float, default=1.)
    parser.add_argument('--domain', type=int, required=True)
    parser.add_argument('--timeout', type=float, default=900.)
    parser.add_argument('--truth', action='store_true')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    os.environ.update(ROS_DOMAIN_ID=str(args.domain), ROS_LOCALHOST_ONLY='1')
    info = bag_info(args.bag)
    processes, logs = [], []
    result = {'mode': 'simulation' if args.truth else 'bag', 'success': False,
              'configuration': yaml.safe_load(args.config.read_text()),
              'bag': info, 'rate': args.rate, 'domain': args.domain,
              'evaluation_kind': 'Fixed-input ROS replay; rate is computational load, not robot speed'}

    def launch(name, command):
        log = (args.output / (name + '.log')).open('w')
        logs.append(log)
        child = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(child)
        return child

    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = Observer(False)
    started = time.monotonic()
    usage_samples = []

    def observe_usage(process):
        try:
            fields = Path('/proc', str(process.pid), 'stat').read_text().rsplit(')', 1)[1].split()
            usage_samples.append({'wall': time.monotonic()-started,
                                  'cpu_seconds': (int(fields[11])+int(fields[12]))/os.sysconf('SC_CLK_TCK'),
                                  'rss_bytes': int(fields[21])*os.sysconf('SC_PAGE_SIZE')})
        except (FileNotFoundError, ProcessLookupError):
            pass

    def pump(duration):
        until = time.monotonic()+duration
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.03)

    def trigger(name):
        client = node.create_client(Trigger, name)
        if not client.wait_for_service(timeout_sec=10.):
            raise RuntimeError('Service unavailable: ' + name)
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic()+30.
        while not future.done() and time.monotonic() < deadline:
            pump(.05)
        if not future.done() or not future.result().success:
            raise RuntimeError('Service failed: ' + name)

    try:
        common = ['--ros-args', '-p', 'use_sim_time:=true']
        slam = launch('slam', [sys.executable, '-c', 'from slam.ros.ros_node import main; main()', *common,
                       '-p', 'config_file:='+str(args.config), '-p', 'mode:=simulation' if args.truth else 'mode:=bag',
                       *(['-p', 'ground_truth_topic:=/ground_truth/odom'] if args.truth else []),
                       '-p', 'output_path:='+str(args.output/'map'), '-p', 'max_path_poses:=20000'])
        launch('tf_filter', [sys.executable, '-c', 'from slam.ros.tf_filter import main; main()', *common])
        pump(3.)
        topics = ['/scan', '/odom', '/tf', '/tf_static']
        if args.truth:
            topics.append('/ground_truth/odom')
        player = launch('player', ['ros2', 'bag', 'play', str(args.bag), '--clock', '--rate', str(args.rate),
                         '--qos-profile-overrides-path', '/worktree/slam/config/bag_qos.yaml',
                         '--topics', *topics, '--remap', '/tf:=/slam/bag_tf', '/tf_static:=/slam/bag_tf_static'])
        stable = None
        last_report = 0.
        last_usage = 0.
        while time.monotonic()-started < args.timeout:
            pump(.1)
            if slam.poll() is not None:
                raise RuntimeError('SLAM process exited')
            if node.status.get('adapter_error'):
                raise RuntimeError(node.status['adapter_error'])
            if time.monotonic()-last_usage > 1.:
                observe_usage(slam)
                last_usage = time.monotonic()
            at_end = node.last_stamp is not None and node.last_stamp >= info['last_scan_stamp']-1e-5
            if player.poll() is not None and node.status.get('queued_scans') == 0:
                stable = time.monotonic() if stable is None else stable
                if time.monotonic()-stable > 3.:
                    result['success'] = bool(at_end)
                    result['finish_reason'] = 'bag_end' if at_end else 'missed_final_scan'
                    break
            else:
                stable = None
            if time.monotonic()-last_report > 20.:
                print(json.dumps({'wall_s': round(time.monotonic()-started), 'status': node.status}), flush=True)
                last_report = time.monotonic()
        else:
            result['finish_reason'] = 'timeout'
        trigger('/slam/pause')
        trigger('/slam/save_map')
        pump(2.)
        result.update(final_status=node.status, paths=node.paths, status_history=node.status_history,
                      scan_messages_observed=len(node.scans), wall_seconds=time.monotonic()-started)
        observe_usage(slam)
        result['slam_process_usage'] = {
            'cpu_seconds': usage_samples[-1]['cpu_seconds'] if usage_samples else None,
            'sampled_peak_rss_bytes': max((s['rss_bytes'] for s in usage_samples), default=None),
            'samples': usage_samples}
    except BaseException as exc:
        result['error'] = str(exc)
        raise
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=10.)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=5.)
        result['exit_codes'] = [p.returncode for p in processes]
        result.setdefault('final_status', node.status)
        result.setdefault('paths', node.paths)
        (args.output / 'samples.json').write_text(json.dumps(result, indent=2)+'\n')
        if node.scans:
            np.savez_compressed(args.output / 'scans.npz',
                                metadata=np.asarray([row[:-1] for row in node.scans]),
                                ranges=np.asarray([row[-1] for row in node.scans]))
        for log in logs:
            log.close()
        with suppress(Exception):
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(json.dumps({key: result.get(key) for key in ('success', 'finish_reason', 'wall_seconds', 'final_status')}), flush=True)


if __name__ == '__main__':
    main()
