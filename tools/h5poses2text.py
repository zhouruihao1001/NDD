import h5py
import sys


def print_usage():
    print("usage: python3 h5poses2text.py <input_file> <out_file>")

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print_usage()
        exit(0)

try:
    hdf5_filename = str(sys.argv[1])
    f = h5py.File(hdf5_filename, 'r')
    print("Found: ", list(f.keys()), "Extracting poses")

    txt_filename = str(sys.argv[2])
    poses_txt = open(txt_filename, 'a')
    print("Will write poses to: ", poses_txt.name)

    g = f['poses']
    sorted_poses = sorted(g.items(), key=lambda x: int(x[0]))

    for elem in sorted_poses:
        pose_group = elem[1]
        for sub_g in pose_group.values():
            pose = list(sub_g)
            poses_txt.write(", ".join(map(str, pose)) + '\n')

    poses_txt.close()
except IOError:
    print("Couldn't open file")
