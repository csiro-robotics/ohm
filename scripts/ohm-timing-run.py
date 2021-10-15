#!/usr/bin/python3
import argparse
import copy
import subprocess


def parse_args():
    parser = argparse.ArgumentParser(description='ohm timing util.')
    parser.add_argument('--cloud', type=str, help='Point cloud file')
    parser.add_argument('--traj', type=str, help='Trajectory file')
    parser.add_argument('--compute', type=str, nargs='+', default=['cpu'], choices=['cpu', 'cuda', 'ocl', 'oct'],
                        help='Compute type(s) to use for the run')
    parser.add_argument('--occ', type=str, nargs='+', default=['occ'], choices=['mean', 'occ', 'ndt', 'ndt-tm'],
                        help='Occupancy type(s) to use for the run')
    parser.add_argument('--resolution', type=float, default=0.1, help='Map resolution')
    parser.add_argument('--ocl-vendor', type=str, nargs='+',
                        default=['none'], help='OpenCL accelerator vendor type(s).')
    parser.add_argument('--gpu-segment-min', type=float, default=0, help='Minimum GPU ray segment to try')
    parser.add_argument('--gpu-segment-max', type=float, default=0, help='Maximum GPU ray segment to try')
    parser.add_argument('--gpu-segment-step', type=float, default=5.0, help='GPU ray segment increment')
    parser.add_argument('--dry-run', action='store_true', help='Show command lines, but do not execute')

    args = parser.parse_args()
    return args


class RunDef:
    def __init__(self):
        self.cloud = None
        self.traj = None
        self.compute = 'cpu'
        self.occupancy = 'occ'
        self.resolution = 0.1
        self.ocl_vendor = None
        self.gpu_segment_length = 0
        self.dry_run = False

    def execute(self):
        program_name = 'ohmpop{}'.format(self.compute) if self.compute != 'oct' else 'octopop'
        out_name = self.compute
        if self.compute == 'ocl' and self.ocl_vendor is not None:
            out_name += '-{}'.format(self.ocl_vendor)
        out_name += '-{}-r{}cm'.format(self.occupancy, int(self.resolution * 100))
        if self.gpu_segment_length > 0:
            out_name += '-s{}m'.format(int(self.gpu_segment_length))

        args = [program_name, self.cloud, self.traj, out_name, '--save-info', '--preload', '--serialise=false']
        args.append('--resolution={}'.format(self.resolution))

        if self.compute == 'ocl' and self.ocl_vendor:
            args.append('--vendor={}'.format(self.ocl_vendor))

        if self.gpu_segment_length > 0:
            args.append('--gpu-ray-segment-length={}'.format(self.gpu_segment_length))

        if self.occupancy == 'mean':
            args.append('--voxel-mean')
        elif self.occupancy == 'ndt':
            args.append('--ndt')
        elif self.occupancy == 'ndt-tm':
            args.append('--ndt=tm')

        if not self.dry_run:
            proc = subprocess.Popen(' '.join(args), shell=True)
            proc.wait()
        else:
            print(' '.join(args))


def float_range(start, stop, step):
    while start < stop:
        yield float(start)
        start += float(step)


if __name__ == '__main__':
    args = parse_args()

    # Build run list
    runs = []

    for compute in args.compute:
        run = RunDef()
        run.cloud = args.cloud
        run.traj = args.traj
        run.resolution = args.resolution
        run.dry_run = args.dry_run
        run.gpu_segment_length = 0

        run.compute = compute

        if compute == 'oct':
            run.occupancy = 'occ'
            runs.append(copy.copy(run))
            break

        for occ in args.occ:
            run.occupancy = occ
            if compute == 'cpu' or compute == 'oct':
                runs.append(copy.copy(run))
            else:
                # GPU based run. Need to add the segment lengths
                segments = list(float_range(args.gpu_segment_min, args.gpu_segment_max +
                                            args.gpu_segment_step * 0.5, args.gpu_segment_step))
                if len(segments) == 0:
                    segments = [0]
                ocl_vendors = args.ocl_vendor if compute == 'ocl' else ['none']
                for ocl_vendor in ocl_vendors:
                    run.ocl_vendor = ocl_vendor if ocl_vendor != 'none' else None
                    for segment in segments:
                        run.gpu_segment_length = segment
                        runs.append(copy.copy(run))

    # execute the runs
    for run in runs:
        run.execute()
    print(len(runs), 'items completed')
