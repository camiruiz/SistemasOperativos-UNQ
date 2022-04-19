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
        log.logger.info("Dispatcher load {pId}".format(pId = pcb.pcbId) )

class PCB():

    def __init__(self, pId, baseDir, limit):
        self._PCBId = pId
        self._status = State.New
        self._baseDir = baseDir
        self._limit = limit
        self._pc = 0

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

    def createPcb(self, baseDir, limit):
        pId = len(self.pcbTable)
        pcb = PCB(pId, baseDir, limit)
        self.pcbTable.append(pcb)
        return pcb

    def hasRunning(self):
        return not(self.running == None)

    @property
    def pcbTable(self):
        return self._pcb_table

    @property
    def running(self):
        return self._running

    @running.setter
    def running(self, value):
        self._running = value

class ReadyQueue():

    def __init__(self):
        self._ready_queue = []

    def add(self, pcb):
        self._ready_queue.append(pcb)

    def isEmpty(self):
        return len(self._ready_queue) == 0

    def getNext(self):
        if self.isEmpty():
            return None
        else:
            return self._ready_queue.pop(0)


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
        prg = irq.parameters
        # cargar el programa en memoria
        baseDir = self.kernel.loader.load(prg)
        limit = len(prg.instructions) - 1
        # crear el pcb
        pcb = self.kernel.pcbTable.createPcb(baseDir, limit)
        # verificar si el cpu esta libre
        if not self.kernel.pcbTable.hasRunning():
            self.kernel.pcbTable.running = pcb
            pcb.status = State.Running
            self.kernel.dispatcher.load(pcb)
        else:
            pcb.status = State.Ready
            self.kernel.readyQueue.add(pcb)

#KIL
class KillInterruptionHandler(AbstractInterruptionHandler):

    def execute(self, irq):
        running = self.kernel.pcbTable.running
        self.kernel.dispatcher.save(running)
        running.status = State.Terminated
        self.kernel.pcbTable.running = None

        nextPCB = self.kernel.readyQueue.getNext()
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

        nextPCB = self.kernel.readyQueue.getNext()
        if not nextPCB is None:
            nextPCB.status = State.Running
            self.kernel.pcbTable.running = nextPCB
            self.kernel.dispatcher.load(nextPCB)

#IOOUT
class IoOutInterruptionHandler(AbstractInterruptionHandler):

    def execute(self, irq):
        pcb = self.kernel.ioDeviceController.getFinishedPCB()
        log.logger.info("------{x}".format(x= self.kernel.pcbTable.hasRunning()))

        if self.kernel.pcbTable.hasRunning():
            pcb.state = State.Ready
            self.kernel.readyQueue.add(pcb)
        else:
            pcb.status = State.Running
            self.kernel.pcbTable.running = pcb
            self.kernel.dispatcher.load(pcb)


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

        ## controls the Hardware's I/O Device
        self._ioDeviceController = IoDeviceController(HARDWARE.ioDevice)
        self._readyQueue = ReadyQueue()
        self._dispatcher = Dispatcher()
        self._pcbTable = PCBTable()
        self._loader = Loader()

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
    def readyQueue(self):
        return self._readyQueue

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
    def run(self, program):
        newIRQ = IRQ(NEW_INTERRUPTION_TYPE, program)
        self._interruptVector.handle(newIRQ)


    def __repr__(self):
        return "Kernel "
