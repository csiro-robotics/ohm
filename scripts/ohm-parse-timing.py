#!/usr/bin/python3

import glob
import os.path
import re

content_res_expr = re.compile(
    r'^Map resolution: ([0-9\.]+)$', re.MULTILINE)
content_mean_expr = re.compile(
    r'^Voxel mean position: (on|off)$', re.MULTILINE)
content_ndt_expr = re.compile(
    r'^NDT map enabled:$', re.MULTILINE)
content_seg_len_expr = re.compile(
    r'^Gpu max ray segment: ([0-9]+)B?$', re.MULTILINE)
content_time_expr = re.compile(
    r'^Total processing time: ([0-9\.]+)s$', re.MULTILINE)
filename_info = re.compile(
    r'(cpu|cuda|ocl)(-(intel|nvidia))?(-(mean|ndt|occ))?(-r([0-9]+)-s([0-9]+))?.*\.txt')


def get_info_item(info, expr, group, default=None):
    first = True
    duplicates = False
    for item in expr.finditer(info):
        if first:
            return item.group(group)
        else:
            duplicates = True
            break
        first = False
    if not duplicates:
        if default is not None:
            return default
        raise RuntimeError('Unable to find content for expression', expr)
    raise RuntimeError('Multiple content items for expression', expr)


def pull_timing(file_path, table):
    with open(file_path, 'r') as data_file:
        try:
            filename = os.path.basename(file_path)
            content = data_file.read()
            # resolution = float(get_info_item(content, content_res_expr, 1))
            mean = get_info_item(content, content_mean_expr, 1)
            mean = True if mean == 'on' else False
            ndt = get_info_item(content, content_ndt_expr, 0, False)
            ndt = True if isinstance(ndt, str) else False
            segment_length = int(get_info_item(
                content, content_seg_len_expr, 1, default=0))
            timing = float(get_info_item(content, content_time_expr, 1))
            run_type = get_info_item(filename, filename_info, 1)
            ocl_gpu_type = get_info_item(filename, filename_info, 3, '')

            occupancy_type = 'occ'
            if mean:
                occupancy_type = 'mean'
            if ndt:
                occupancy_type = 'ndt'

            if ocl_gpu_type:
                run_type = '{} {}'.format(run_type, ocl_gpu_type)

            if segment_length:
                run_type = '{} s{:02d}m'.format(run_type, segment_length)

            if run_type not in table:
                table[run_type] = {}

            table[run_type][occupancy_type] = timing
        except RuntimeError:
            print('Skipping {} - failed extraction'.format(file_path))


if __name__ == '__main__':
    # with open('timings.csv', 'w') as out:

    table = {}
    data_files = glob.glob('*.txt')

    for data_file in data_files:
        # print(data_file)
        pull_timing(data_file, table)

    delimit = ','
    with open('timings.csv', 'w') as out:
        def write_result(out, name, data_line):
            if name in data_line:
                out.write(str(data_line[name]))
            else:
                out.write('')

        out.write('Run Type' + delimit + 'occ' + delimit + 'mean' + delimit + 'ndt\n')
        keys = list(table.keys())
        keys.sort()
        for run_type in keys:
            run_data = table[run_type]
            out.write(run_type)
            out.write(delimit)
            write_result(out, 'occ', run_data)
            out.write(delimit)
            write_result(out, 'mean', run_data)
            out.write(delimit)
            write_result(out, 'ndt', run_data)
            out.write('\n')
