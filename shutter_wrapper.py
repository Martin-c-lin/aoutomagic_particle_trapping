import instruments

class shutter():

    def __init__(self, port='COM1'):
        shutter = instruments.thorlabs.SC10.open_serial(port=port, baud=9600, vid=None, pid=None, serial_number=None, timeout=2, write_timeout=10)

    def open_for_duration(self,duration):
        try:
            tmp.sendcmd(cmd='open='+str(duration))
        except:
            pass
            #print()
        try:
            tmp.sendcmd(cmd='ens')
        except:
            pass
            # print('Ens message sent')
