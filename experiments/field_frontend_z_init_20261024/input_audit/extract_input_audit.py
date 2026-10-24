#!/usr/bin/env python3
"""Read-only SQLite CDR prefix extraction; never reads an entire cloud BLOB.

Indices follow metadata file order, then (record timestamp, SQLite message id).
raw_message_order includes every topic. sensor_input_order includes the two
sensor topics. raw_scan_id and raw_imu_id are independent 0-based counters.
The header stamp is an integer from the original sec/nanosec fields; point
timestamps are the original doubles and cannot establish sub-double precision.
"""
from __future__ import annotations

import argparse
import collections
import csv
import json
import math
from pathlib import Path
import sqlite3
import struct


class CDR:
    def __init__(self, blob):
        self.blob = blob
        if blob[:4] != b'\x00\x01\x00\x00':
            raise ValueError(f'Expected little-endian CDR encapsulation, got {blob[:4]!r}')
        self.p = 4

    def read(self, fmt, alignment=None):
        size = struct.calcsize('<' + fmt)
        alignment = alignment or size
        self.p = 4 + ((self.p - 4 + alignment - 1) // alignment) * alignment
        out = struct.unpack_from('<' + fmt, self.blob, self.p)
        self.p += size
        return out[0] if len(out) == 1 else out

    def string(self):
        size = self.read('I')
        out = self.blob[self.p:self.p + size]
        if len(out) != size or out[-1:] != b'\0':
            raise ValueError('Truncated or invalid CDR string')
        self.p += size
        return out[:-1].decode('utf-8')

    def header(self):
        sec, ns = self.read('i'), self.read('I')
        if sec < 0 or ns >= 1_000_000_000:
            raise ValueError(f'Invalid original stamp: {sec}, {ns}')
        return sec * 1_000_000_000 + ns, self.string()


def parse_imu(prefix):
    r = CDR(prefix)
    stamp, frame = r.header()
    q = r.read('4d', 8)
    orientation_cov = r.read('9d', 8)
    gyro = r.read('3d', 8)
    gyro_cov = r.read('9d', 8)
    acc = r.read('3d', 8)
    acc_cov = r.read('9d', 8)
    return dict(header_ns=stamp, frame_id=frame, q=q, orientation_cov=orientation_cov,
                gyro=gyro, gyro_cov=gyro_cov, acc=acc, acc_cov=acc_cov)


def parse_cloud(prefix):
    r = CDR(prefix)
    stamp, frame = r.header()
    height, width = r.read('I'), r.read('I')
    fields = {}
    for _ in range(r.read('I')):
        name = r.string()
        fields[name] = (r.read('I'), r.read('B'), r.read('I'))
    endian = r.read('B')
    point_step, row_step = r.read('I'), r.read('I')
    data_size = r.read('I')
    if endian or height != 1 or row_step != width * point_step or data_size != row_step:
        raise ValueError('This audit requires the observed little-endian unorganized cloud layout')
    for name, expected in {'x': (0, 7, 1), 'y': (4, 7, 1), 'z': (8, 7, 1),
                           'timestamp': (18, 8, 1)}.items():
        if fields.get(name) != expected:
            raise ValueError(f'Unexpected {name} PointField: {fields.get(name)}')
    if point_step != 26 or width == 0:
        raise ValueError('Unexpected cloud point_step or empty cloud')
    point0 = struct.unpack_from('<d', prefix, r.p + fields['timestamp'][0])[0]
    return dict(header_ns=stamp, frame_id=frame, height=height, width=width,
                point_step=point_step, data_offset=r.p, data_size=data_size,
                point0_timestamp_s=point0, fields=fields)


def nearest_ns(value):
    numerator, denominator = value.as_integer_ratio()
    q, remainder = divmod(numerator * 1_000_000_000, denominator)
    return q + int(2 * remainder > denominator or
                   (2 * remainder == denominator and q % 2))


def float32(value):
    return struct.unpack('<f', struct.pack('<f', value))[0]


def sample_tail(connection, message_id, total_size, cloud, filter_num=6, blind=.5):
    # 8192 bytes covers about 315 trailing points, not the full 3 MB cloud.
    blob = connection.execute('SELECT substr(data,-8192) FROM messages WHERE id=?',
                              (message_id,)).fetchone()[0]
    tail_start = total_size - len(blob)
    def point(index):
        offset = cloud['data_offset'] + index * cloud['point_step'] - tail_start
        if offset < 0:
            raise ValueError('No retained point found within bounded tail sample')
        x, y, z = struct.unpack_from('<3f', blob, offset)
        stamp = struct.unpack_from('<d', blob, offset + 18)[0]
        return x, y, z, stamp
    last_index = cloud['width'] - 1
    last_timestamp = point(last_index)[3]
    for index in range(last_index - last_index % filter_num, -1, -filter_num):
        x, y, z, timestamp = point(index)
        squared_range = float32(float32(float32(x*x) + float32(y*y)) + float32(z*z))
        if not squared_range < blind * blind:
            curvature = float32((timestamp - cloud['point0_timestamp_s']) * 1000.)
            # rclcpp::Time::seconds() converts the integer ns to double then *1e-9.
            begin_s = float(cloud['header_ns']) * 1e-9
            end_s = begin_s + curvature / 1000.
            return dict(last_raw_point_index=last_index, last_raw_timestamp_s=last_timestamp,
                        filtered_last_point_index=index, filtered_last_timestamp_s=timestamp,
                        curvature_ms_float32=curvature, span_s=curvature / 1000.,
                        predicted_scan_end_double_s=end_s,
                        predicted_scan_end_nearest_ns=round(end_s * 1e9),
                        header_minus_point0_nearest_ns=cloud['header_ns'] - nearest_ns(cloud['point0_timestamp_s']))
    raise ValueError('Cloud has no retained tail point')


def gap_summary(stamps, threshold_ns):
    delta = [(i, stamps[i] - stamps[i-1]) for i in range(1, len(stamps))]
    return dict(count=len(stamps), first_header_ns=stamps[0], last_header_ns=stamps[-1],
                header_span_s=(stamps[-1]-stamps[0])*1e-9,
                nonmonotonic_count=sum(d < 0 for _, d in delta),
                repeated_header_count=sum(d == 0 for _, d in delta),
                max_header_gap_ns=max(d for _, d in delta),
                gaps_over_threshold=[dict(after_raw_id=i, before_header_ns=stamps[i-1],
                                         after_header_ns=stamps[i], gap_ns=d)
                                     for i, d in delta if d > threshold_ns])


def extract(bag, output):
    output.mkdir(parents=True, exist_ok=True)
    targets = [output / name for name in ['imu_raw.csv', 'cloud_timing.csv',
                'cloud_tail_samples.csv', 'input_gap_summary.json', 'extraction_manifest.json']]
    if any(p.exists() for p in targets):
        raise FileExistsError('Refusing to overwrite any extraction artifact')
    files = sorted(bag.glob('bag_*.db3'), key=lambda p: int(p.stem.split('_')[-1]))
    if not files:
        raise FileNotFoundError(bag)
    covariance_counts = {k: collections.Counter() for k in ['orientation_cov', 'gyro_cov', 'acc_cov', 'q']}
    imu_stamps, cloud_stamps, point0_delta, tails, partitions = [], [], [], [], []
    message_order = sensor_order = 0
    sample_ids = {0, 1, 2, 100, 4093, 4094, 4095, 8185, 8186, 8187}
    common = ['raw_message_order', 'sensor_input_order', 'partition', 'sqlite_message_id', 'record_ns', 'header_ns', 'frame_id']
    imu_columns = ['raw_imu_id', *common, 'header_occurrence', 'header_multiplicity', 'source_gap_ns',
                   'ax', 'ay', 'az', 'gx', 'gy', 'gz']
    cloud_columns = ['raw_scan_id', *common, 'header_occurrence', 'header_multiplicity', 'source_gap_ns',
                     'width', 'point_step', 'point0_timestamp_s', 'point0_nearest_ns', 'header_minus_point0_nearest_ns']
    # Prefix extraction is retained in memory only until multiplicities are known.
    imu_rows, cloud_rows = [], []
    for file in files:
        con = sqlite3.connect(file.as_uri() + '?mode=ro&immutable=1', uri=True)
        con.execute('PRAGMA query_only=ON')
        topics = dict(con.execute('SELECT id,name FROM topics'))
        ids = [key for key, name in topics.items() if name in ['/lidar_imu', '/lidar_points']]
        query = 'SELECT id,timestamp,topic_id,CASE WHEN topic_id IN (' + ','.join('?' for _ in ids) + ') THEN substr(data,1,512) ELSE NULL END,length(data) FROM messages ORDER BY timestamp,id'
        file_count = 0
        for sqlite_id, record_ns, topic_id, prefix, total_size in con.execute(query, ids):
            name = topics[topic_id]
            if prefix is not None:
                data = parse_imu(prefix) if name == '/lidar_imu' else parse_cloud(prefix)
                stamp = data['header_ns']
                base = dict(raw_message_order=message_order, sensor_input_order=sensor_order,
                            partition=file.name, sqlite_message_id=sqlite_id,
                            record_ns=record_ns, header_ns=stamp, frame_id=data['frame_id'])
                if name == '/lidar_imu':
                    row = dict(raw_imu_id=len(imu_stamps), **base,
                               source_gap_ns=stamp-imu_stamps[-1] if imu_stamps else '')
                    row.update(zip(['ax', 'ay', 'az', 'gx', 'gy', 'gz'], [*data['acc'], *data['gyro']]))
                    imu_rows.append(row); imu_stamps.append(stamp)
                    for k in covariance_counts:
                        covariance_counts[k][data[k]] += 1
                else:
                    row = dict(raw_scan_id=len(cloud_stamps), **base,
                               source_gap_ns=stamp-cloud_stamps[-1] if cloud_stamps else '',
                               width=data['width'], point_step=data['point_step'],
                               point0_timestamp_s=data['point0_timestamp_s'],
                               point0_nearest_ns=nearest_ns(data['point0_timestamp_s']))
                    row['header_minus_point0_nearest_ns'] = stamp-row['point0_nearest_ns']
                    if row['raw_scan_id'] in sample_ids:
                        tails.append(dict(raw_scan_id=row['raw_scan_id'], header_ns=stamp,
                                          **sample_tail(con, sqlite_id, total_size, data)))
                    cloud_rows.append(row); cloud_stamps.append(stamp)
                    point0_delta.append(row['header_minus_point0_nearest_ns'])
                sensor_order += 1
            message_order += 1; file_count += 1
        con.close()
        partitions.append(dict(path=file.name, size_bytes=file.stat().st_size, message_count=file_count))
        print(json.dumps(dict(partition=file.name, imu_count=len(imu_stamps), cloud_count=len(cloud_stamps))), flush=True)
    for path, rows, columns in [(targets[0], imu_rows, imu_columns), (targets[1], cloud_rows, cloud_columns)]:
        multiplicity = collections.Counter(r['header_ns'] for r in rows)
        occurrences = collections.Counter()
        with path.open('x') as stream:
            writer = csv.DictWriter(stream, columns); writer.writeheader()
            for row in rows:
                stamp = row['header_ns']
                row['header_occurrence'] = occurrences[stamp]
                row['header_multiplicity'] = multiplicity[stamp]
                occurrences[stamp] += 1; writer.writerow(row)
    with targets[2].open('x') as stream:
        writer = csv.DictWriter(stream, list(tails[0])); writer.writeheader(); writer.writerows(tails)
    report = dict(schema='original_bag_input_small_fields_v1', bag=str(bag),
                  imu=gap_summary(imu_stamps, 7_500_000),
                  cloud=gap_summary(cloud_stamps, 150_000_000),
                  point0_header_relation=dict(min_header_minus_point0_nearest_ns=min(point0_delta),
                                             max_header_minus_point0_nearest_ns=max(point0_delta),
                                             all_abs_difference_at_most_256ns=all(abs(x)<=256 for x in point0_delta)),
                  imu_covariance_and_orientation={k:[dict(value=list(v), count=c) for v,c in counter.items()]
                                                  for k,counter in covariance_counts.items()},
                  source_message_counts_match_expected=len(imu_stamps)==166023 and len(cloud_stamps)==8188,
                  raw_gaps_preserved=True, inserted_or_interpolated_messages=0,
                  full_point_clouds_read=0, tail_sample_count=len(tails),
                  limitations=['point0 double timestamp has epoch-dependent finite precision',
                               'tail samples check only retained last point; not all point timestamps',
                               'source gaps cannot distinguish device omission from recording omission',
                               'raw unit labels require archived driver contract, not magnitude inference'])
    targets[3].write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    targets[4].write_text(json.dumps(dict(schema='read_only_sqlite_prefix_extraction_v1',
                     bag=str(bag), partitions=partitions, total_messages=message_order,
                     imu_count=len(imu_stamps), cloud_count=len(cloud_stamps),
                     sql_blob_prefix_bytes=512, sampled_tail_bytes=8192,
                     indices='metadata numeric partition order; record timestamp then SQLite id; 0-based',
                     source_files_opened='mode=ro&immutable=1 with PRAGMA query_only=ON',
                     source_mutations=0, ros_context_created=False), indent=2)+'\n')
    print(json.dumps(dict(imu_count=len(imu_stamps), cloud_count=len(cloud_stamps),
                         output=str(output), expected_counts=report['source_message_counts_match_expected'])))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    extract(args.bag.resolve(), args.output.resolve())
