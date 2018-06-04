#!/usr/bin/python
import pysftp
import progressbar


def progress(done, total):
    bar.update(done)

if __name__ == '__main__':
    with pysftp.Connection('robosub.eecs.wsu.edu',
            username='sftp_user',
            password='gocougs',
            default_path='/data/vision/datasets') as sftp:
            total = sftp.stat('dice.json')
            bar = progressbar.ProgressBar(max_value=total.st_size)
            sftp.get('dice.json', callback=progress)
            bar.finish()
