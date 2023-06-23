import matplotlib.pyplot as plt
import random as r


def plot_with_suff(suff: str):
    blobs = []
    with open('./data/blobs_{}.csv'.format(suff), 'r') as f:
        next(f) # skip header
        lineCounter = 0
        for line in f:
            lineCounter += 1
            data = line.rstrip().split(';')
            blob = {'id': str(lineCounter), 'x': [], 'y': []}
            track_data = data[1].split('|')
            for point_data in track_data:
                point = point_data.split(',')
                blob['x'].append(float(point[0]))
                blob['y'].append(float(point[1]))
            blobs.append(blob)

    dpi = 100
    plt.figure(figsize=(720/dpi, 480/dpi), dpi=dpi)
    for blob in blobs:
        hex_color = '#' + ''.join([r.choice('0123456789ABCDEF') for _ in range(6)])
        plt.plot(blob['x'], blob['y'], color=hex_color, label=('Blob #' + blob['id']))

    plt.legend(loc='upper left')
    plt.grid(alpha=0.2)
    ax = plt.gca()
    ax.invert_yaxis()

    plt.title('Simple MOT ({})'.format(suff), fontsize=15, fontweight='bold')
    plt.savefig('./data/mot_simple_{}.png'.format(suff), dpi=dpi)
    # plt.show()

    plt.clf() # clear figure

plot_with_suff('spread')
plot_with_suff('similar')