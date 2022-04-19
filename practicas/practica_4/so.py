#!/usr/bin/env python

from hardware import *
import log

## emulates a compiled program
class Program():

    def __init__(self, name, instructions):
        self._name = name
        self._instructions = self.expand(instructions)

    @property
    def name(self):
        return self._name

    @property
    def instructions(self):
        return self._instructions

    def addInstr(self, instruction):
        self._instructions.append(instruction)

    def expand(self, instructions):
        expanded = []
        for i in instructions:
            if isinstance(i, list):
                ## is a list of instructions
                expanded.extend(i)
            else:
                ## a single instr (a String)
                expanded.append(i)

        ## now test if last instruction is EXIT
        ## if not... add an EXIT as final instruction
        last = expanded[-1]
        if not ASM.isEXIT(last):
            expanded.append(INSTRUCTION_EXIT)

        return expanded

    def __repr__(self):
        return "Program({name}, {instructions})".format(name=self._name, instructions=self._instructions)

#Ejercicio 6.2 implentar
class Loader():

    def __init__(self):
        self._free = 0

    def load(self, prg):
        baseDir = self._free
        for i in prg.instructions:
            HARDWARE.memory.write(self._free, i)
            self._free = self._free + 1
        log.logger.info(HARDWARE.memory)
        return baseDir

class Dispatcher():

    def save(self, pcb):
        pcb.pc = HARDWARE.cpu.pc
        HARDWARE.cpu.pc = -1
        log.logger.info("Dispatcher save {pId}".format(pId = pcb.pcbId) )

    def load(self, pcb):
        HARDWARE.cpu.pc = pcb.pc
        HARDWARE.mmu.baseDir = pcb.baseDir
        HARDWARE.timer.reset()
        log.logger.info("Dispatcher load {pId}".format(pId = pcb.pcbId) )

class PCB():

    def __init__(self, pId, baseDir, limit, prioridad):
        self._PCBId = pId
        self._status = State.New
        self._baseDir = baseDir
        self._limit = limit
        self._pc = 0
        self._prioridad = prioridad

    @property
    def pcbId(self):
        return self._PCBId

    @property
    def status(self):
        return self._status

    @status.setter
    def status(self, state):
        self._status = state

    @property
    def baseDir(self):
        return self._baseDir

    @property
    def limit(self):
        return self._limit

    @property
    def pc(self):
        return self._pc

    @pc.setter
    def pc(self, value):
        self._pc = value

    @property
    def prioridad(self):
        return self._prioridad

class State():

    New = "new"
    Running = "running"
    Ready = "ready"
    Waiting = "waiting"
    Terminated = "terminated"

class PCBTable():

    def __init__(self):
        self._pcb_table = []
        self._running = None

    def createPcb(self, baseDir, limit, prioridad):
        pId = len(self.pcbTable)
        pcb = PCB(pId, baseDir, limit, prioridad)
        self.pcbTable.append(pcb)
        return pcb

    def hasRunning(self):
        return not (self.running == None)

    @property
    def pcbTable(self):
        return self._pcb_table

    @property
    def running(self):
        return self._running

    @running.setter
    def running(self, value):
        self._running = value

class SchedulerFCFS():

    def __init__(self):
        self._readyQueue = []

    def add(self, pcb):
        self._readyQueue.append(pcb)

    def isEmpty(self):
        return len(self._readyQueue) == 0

    def getNext(self):
        if self.isEmpty():
            return None
        else:
            return self._readyQueue.pop(0)

    @property
    def readyQueue(self):
        return self._readyQueue

    @readyQueue.setter
    def readyQueue(self, value):
        self._readyQueue = value

    def debeExpropiar(self, PCBrunning, PCBentrance):
        return False

class SchedulerPrioridadExpropiativo(SchedulerFCFS):

    def debeExpropiar(self, PCBrunning, PCBentrante):
        return PCBrunning.prioridad < PCBentrante.prioridad

    def add(self, pcb):
        listaNueva = []
        newAgedPCB = AgedPCB(pcb)
        while not self.isEmpty() and (self.readyQueue[0].auxPriority > newAgedPCB.auxPriority):
            listaNueva.append(self.readyQueue.pop(0))
        listaNueva.append(newAgedPCB)
        self.readyQueue = listaNueva + self.readyQueue

    def getNext(self):
        if self.isEmpty():
            return None
        else:
            self.readyQueue = list(map(lambda aged: aged.addPriority(), self.readyQueue))
            return self.readyQueue.pop(0).pcb

class SchedulerPrioridadNoExpropiativo(SchedulerPrioridadExpropiativo):

    def __init__(self):
        super().__init__()

    def debeExpropiar(self, PCBruning, PCBentrante):
        return False

class SchedulerRoundRobin(SchedulerFCFS):

    def __init__(self, quantum):
        super().__init__()
        HARDWARE.timer.quantum = quantum

class AgedPCB():

    def __init__(self, pcb):
        self._pcb = pcb
        self._auxPriority = pcb.prioridad

    @property
    def pcb(self):
        return self._pcb

    @property
    def auxPriority(self):
        return self._auxPriority

    @auxPriority.setter
    def auxPriority(self, value):
        self._auxPriority = value

    def addPriority(self):
        self.auxPriority = self.auxPriority + 1
        return self


## emulates an Input/Output device controller (driver)
class IoDeviceController():

    def __init__(self, device):
        self._device = device
        self._waiting_queue = []
        self._currentPCB = None

    def runOperation(self, pcb, instruction):
        pair = {'pcb': pcb, 'instruction': instruction}
        # append: adds the element at the end of the queue
        self._waiting_queue.append(pair)
        # try to send the instruction to hardware's device (if is idle)
        self.__load_from_waiting_queue_if_apply()

    def getFinishedPCB(self):
        finishedPCB = self._currentPCB
        self._currentPCB = None
        self.__load_from_waiting_queue_if_apply()
        return finishedPCB

    def __load_from_waiting_queue_if_apply(self):
        if (len(self._waiting_queue) > 0) and self._device.is_idle:
            ## pop(): extracts (deletes and return) the first element in queue
            pair = self._waiting_queue.pop(0)
            #print(pair)
            pcb = pair['pcb']
            instruction = pair['instruction']
            self._currentPCB = pcb
            self._device.execute(instruction)


    def __repr__(self):
        return "IoDeviceController for {deviceID} running: {currentPCB} waiting: {waiting_queue}".format(deviceID=self._device.deviceId, currentPCB=self._currentPCB, waiting_queue=self._waiting_queue)

## emulates the  Interruptions Handlers
class AbstractInterruptionHandler():
    def __init__(self, kernel):
        self._kernel = kernel

    @property
    def kernel(self):
        return self._kernel

    def execute(self, irq):
        log.logger.error("-- EXECUTE MUST BE OVERRIDEN in class {classname}".format(classname=self.__class__.__name__))

#Las 4 interrupciones:

#NEW
class NewInterruptionHandler(AbstractInterruptionHandler):

    def execute(self, irq):
        prg = irq.parameters["program"]
        # cargar el programa en memoria
        baseDir = self.kernel.loader.load(prg)
        limit = len(prg.instructions) - 1
        # crear el pcb
        pcb = self.kernel.pcbTable.createPcb(baseDir, limit, irq.parameters["priority"])
        # verificar si el cpu esta libre
        if not self.kernel.pcbTable.hasRunning():
            self.kernel.pcbTable.running = pcb
            pcb.status = State.Running
            self.kernel.dispatcher.load(pcb)
        else:
            if self.kernel.scheduler.debeExpropiar(self.kernel.pcbTable.running, pcb):
                pcbRunning = self.kernel.pcbTable.running
                self.kernel.pcbTable.running = None
                self.kernel.dispatcher.save(pcbRunning)
                pcbRunning.status = State.Ready
                self.kernel.scheduler.add(pcbRunning)

                self.kernel.pcbTable.running = pcb
                pcb.status = State.Running
                self.kernel.dispatcher.load(pcb)
            else:
                pcb.status = State.Ready
                self.kernel.scheduler.add(pcb)

#KIL
class KillInterruptionHandler(AbstractInterruptionHandler):

    def execute(self, irq):
        running = self.kernel.pcbTable.running
        self.kernel.dispatcher.save(running)
        running.status = State.Terminated
        self.kernel.pcbTable.running = None

        nextPCB = self.kernel.scheduler.getNext()
        if not nextPCB is None:
            nextPCB.status = State.Running
            self.kernel.pcbTable.running = nextPCB
            self.kernel.dispatcher.load(nextPCB)

#IOIN
class IoInInterruptionHandler(AbstractInterruptionHandler):

    def execute(self, irq):
        running = self.kernel.pcbTable.running
        self.kernel.dispatcher.save(running)
        running.status = State.Waiting
        self.kernel.pcbTable.running = None
        self.kernel.ioDeviceController.runOperation(running, irq)

        nextPCB = self.kernel.scheduler.getNext()
        if not nextPCB is None:
            nextPCB.status = State.Running
            self.kernel.pcbTable.running = nextPCB
            self.kernel.dispatcher.load(nextPCB)

#IOOUT
class IoOutInterruptionHandler(AbstractInterruptionHandler):

    def execute(self, irq):
        pcb = self.kernel.ioDeviceController.getFinishedPCB()
        log.logger.info("------{x}".format(x= self.kernel.pcbTable.hasRunning()))

        if not self.kernel.pcbTable.hasRunning():
            pcb.status = State.Running
            self.kernel.pcbTable.running = pcb
            self.kernel.dispatcher.load(pcb)
        else:
            if self.kernel.scheduler.debeExpropiar(self.kernel.pcbTable.running, pcb):
                self.kernel.dispatcher.save(self.kernel.pcbTable.running)
                self.kernel.scheduler.add(self.kernel.pcbTable.running)
                self.kernel.pcbTable.running.status = State.Ready
                self.kernel.dispatcher.load(pcb)
            else:
                pcb.state = State.Ready
                self.kernel.scheduler.add(pcb)

#TIMEOUT
class TimeoutInterruptionHandler(AbstractInterruptionHandler):

    def execute(self, irq):
        self.kernel.dispatcher.save(self.kernel.pcbTable.running)
        self.kernel.scheduler.add(self.kernel.pcbTable.running)
        self.kernel.pcbTable.running.status = State.Ready
        self.kernel.dispatcher.load(self.kernel.scheduler.getNext())


# emulates the core of an Operative System
class Kernel():

    def __init__(self):
        ## setup interruption handlers
        killHandler = KillInterruptionHandler(self)
        HARDWARE.interruptVector.register(KILL_INTERRUPTION_TYPE, killHandler)

        ioInHandler = IoInInterruptionHandler(self)
        HARDWARE.interruptVector.register(IO_IN_INTERRUPTION_TYPE, ioInHandler)

        ioOutHandler = IoOutInterruptionHandler(self)
        HARDWARE.interruptVector.register(IO_OUT_INTERRUPTION_TYPE, ioOutHandler)

        newHandler = NewInterruptionHandler(self)
        HARDWARE.interruptVector.register(NEW_INTERRUPTION_TYPE, newHandler)

        timeOutHandler = TimeoutInterruptionHandler(self)
        HARDWARE.interruptVector.register(TIMEOUT_INTERRUPTION_TYPE, timeOutHandler)

        ## controls the Hardware's I/O Device
        self._ioDeviceController = IoDeviceController(HARDWARE.ioDevice)
        self._dispatcher = Dispatcher()
        self._pcbTable = PCBTable()
        self._loader = Loader()
        self._scheduler = SchedulerRoundRobin(3)
        self._interruptVector = HARDWARE.interruptVector

    @property
    def ioDeviceController(self):
        return self._ioDeviceController

    ##
    def load_program(self, program):
        # loads the program in main memory
        progSize = len(program.instructions)
        for index in range(0, progSize):
            inst = program.instructions[index]
            HARDWARE.memory.write(index, inst)
    ##

    @property
    def scheduler(self):
        return self._scheduler

    @property
    def dispatcher(self):
        return self._dispatcher

    @property
    def pcbTable(self):
        return self._pcbTable

    @property
    def loader(self):
        return self._loader

    @property
    def interruptVector(self):
        return self._interruptVector


#6.1: Implementar la interrupción #NEW

    ## emulates a "system call" for programs execution
    def run(self, program, priority):
        newIRQ = IRQ(NEW_INTERRUPTION_TYPE, {"program":program, "priority":priority})
        self._interruptVector.handle(newIRQ)


    def __repr__(self):
        return "Kernel "
