import sys
import time
from argparse import ArgumentParser, SUPPRESS
import psutil
import matplotlib.pyplot as plt

class SystemLogger:
    def __init__(self, args):
        self.pid  = int(args.pid)
        self.logfile  = args.log
        self.plot = args.plot
        self.duration = args.duration
        self.interval = args.interval
        self.pr = psutil.Process(self.pid)
        self.logFormat()
        
    def logFormat(self):
        self.log = {}
        self.log['times'] = []
        self.log['cpu'] = []
        self.log['mem_real'] = []
        self.log['mem_virtual'] = []

    def monitor(self):
        # Record start time
        start_time = time.time()
        self.file = open(self.logfile, 'w')
        self.file.write("# {0:12s} {1:12s} {2:12s} {3:12s}\n".format(
            'Elapsed time'.center(12),
            'CPU (%)'.center(12),
            'Real (MB)'.center(12),
            'Virtual (MB)'.center(12))
        )
        
        try:
            while True:
                # Find current time
                current_time = time.time()

                try:
                    pr_status = self.pr.status()
                except TypeError:  # psutil < 2.0
                    pr_status = self.pr.status
                except psutil.NoSuchProcess:  # pragma: no cover
                    break
                
                # Check if process status indicates we should exit
                if pr_status in [psutil.STATUS_ZOMBIE, psutil.STATUS_DEAD]:
                    print("Process finished ({0:.2f} seconds)"
                        .format(current_time - start_time))
                    break
                
                # Check if we have reached the maximum time
                if self.duration is not None and current_time - start_time > self.duration:
                    print("Reached the duration and exit program")
                    break
                
                # Get current CPU and memory
                current_cpu =  self.pr.cpu_percent()
                current_mem = self.pr.memory_info()
                current_mem_real = current_mem.rss / 1024. ** 2
                current_mem_virtual = current_mem.vms / 1024. ** 2

                self.file.write("{0:12.3f} {1:12.3f} {2:12.3f} {3:12.3f}\n".format(
                    current_time - start_time,
                    current_cpu,
                    current_mem_real,
                    current_mem_virtual))
                self.file.flush()

                if self.interval is not None:
                    time.sleep(self.interval)

                self.log['times'].append(current_time - start_time)
                self.log['cpu'].append(current_cpu)
                self.log['mem_real'].append(current_mem_real)
                self.log['mem_virtual'].append(current_mem_virtual)
        except KeyboardInterrupt:  # pragma: no cover
            pass

        with plt.rc_context({'backend': 'Agg'}):
            fig = plt.figure()
            ax = fig.add_subplot(1, 1, 1)
            ax.plot(self.log['times'], self.log['cpu'], '-', lw=1, color='r')
            ax.set_ylabel('CPU (%)', color='r')
            ax.set_xlabel('time (s)')
            ax.set_ylim(0., max(self.log['cpu']) * 1.2)
            ax2 = ax.twinx()
            ax2.plot(self.log['times'], self.log['mem_real'], '-', lw=1, color='b')
            ax2.set_ylim(0., max(self.log['mem_real']) * 1.2)
            ax2.set_ylabel('Real Memory (MB)', color='b')
            ax.grid()
            fig.savefig(self.plot)

        self.file.close()
    @staticmethod
    def build_argparser():
        parser = ArgumentParser(add_help=False)
        args = parser.add_argument_group("Options")
        args.add_argument('--pid', type=str,
                            help='the process id or command')

        args.add_argument('--log', type=str,
                            help='output the statistics to a file')

        args.add_argument('--plot', type=str,
                            help='output the statistics to a plot')

        args.add_argument('--duration', type=float,
                            help='how long to record for (in seconds). If not '
                                    'specified, the recording is continuous until '
                                    'the job exits.')

        args.add_argument('--interval', type=float,
                            help='how long to wait between each sample (in '
                                    'seconds). By default the process is sampled '
                                    'as often as possible.')

        args.add_argument('--include-children',
                            help='include sub-processes in statistics (results '
                                    'in a slower maximum sampling rate).',
                            action='store_true')

        return parser

def main():
    args = SystemLogger.build_argparser().parse_args()
    pid = int(args.pid)

    # Check the PID exists
    if psutil.pid_exists(pid):
        systemLogger = SystemLogger(args)
        systemLogger.monitor()

    else:
        print("Process ID {} doesnt exists\n".format(pid))
    

if __name__ == '__main__':
    sys.exit(main() or 0)