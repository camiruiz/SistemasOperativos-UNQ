from hardware import *
from so import *
import log


##
##  MAIN 
##
if __name__ == '__main__':
    log.setupLogger()
    log.logger.info('Starting emulator')

    ## setup our hardware and set memory size
    HARDWARE.setup(40)
    HARDWARE.mmu.frameSize = 4

    ## Switch on computer
    HARDWARE.switchOn()

    ## new create the Operative System Kernel
    # "booteamos" el sistema operativo
    kernel = KernelFactory()\
        .RoundRobin(3)\
        .Paginated()\
        .Create()

    # Ahora vamos a intentar ejecutar 3 programas a la vez
    ##################
    kernel.fileSystem.write("c:/prg1.exe", [ASM.CPU(2), ASM.IO(), ASM.CPU(3), ASM.IO(), ASM.CPU(2)])
    kernel.fileSystem.write("c:/prg2.exe", [ASM.CPU(7)])
    kernel.fileSystem.write("c:/prg3.exe", [ASM.CPU(4), ASM.IO(), ASM.CPU(1)])

    # execute all programs "concurrently"
    kernel.run("c:/prg2.exe", 4)
    kernel.run("c:/prg3.exe", 10)
    kernel.run("c:/prg1.exe", 5)





