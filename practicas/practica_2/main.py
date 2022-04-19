from hardware import *
from so import *
import log


##
##  MAIN 
##
if __name__ == '__main__':
    log.setupLogger()
    log.logger.info('Starting emulator')

    ## setup our hardware and set memory size to 20 "cells"
    HARDWARE.setup(20)

    ## Switch on computer
    HARDWARE.switchOn()

    ## new create the Operative System Kernel
    # "booteamos" el sistema operativo
    kernel = Kernel()

    ##  create a program
    prg1 = Program("prg1.exe", [ASM.CPU(2), ASM.IO(), ASM.CPU(3)])
    prg2 = Program("prg2.exe", [ASM.CPU(4), ASM.IO(), ASM.CPU(1)])
    prg3 = Program("prg3.exe", [ASM.CPU(3)])

    batch = [prg1, prg2, prg3]

    # execute the program
    kernel.executeBatch(batch)


    ## Punto 1:  Entender las clases InterruptVector() y Clock() y poder explicar cómo funcionan.
    ## InterruptVector(): 
    ##-register: registra las direcciones de las rutinas del software cuando se solicita una interrupción.
    ##-handle: se encarga de la interrupción.

    ##Clock() : inicia una lista vacía de sus suscriptores y un estado running que se inicia como False
    ##-addSubscriber: suma a la lista un suscriptor
    ##-stop: modifica el estado de running a false
    ##-start: arranca el thread de ticks

    ## Punto 2:Explicar cómo se llegan a ejecutar KillInterruptionHandler.execute().

    ##El kernel, cuando la instrucción es una salida, se encarga de ejecutar la interrupción.



