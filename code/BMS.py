# Battery Management System Embedded OS
# https://github.com/AdinAck/Fantasy-Bike/tree/master/BMS
# Please feel free to post issues or questions on the GitHub repository.
# By Adin Ackerman and Artin Kim
# ======================================================================================================================

# NVM Index:
# 0x0: 0 if measurement/battery fault has not been detected
#      1 if measurement/battery fault has been detected

# SERCOM usage:
# Writing:

# 1xxxxxxx + xxxxxxxx + ...
# Instruction + Data (up to 63 bytes)

# Receive:
# 11111111 If successful



# Reading:
# 0xxxxxxx
# Instruction

# Receive:
# xxxxxxxx + ...
# Data



# SERCOM Instructions:
# Writing:
# Bits     | Description                                    | Value type
# ===========================================================================
# 10000000 = Mode                                             Integer 0,1,2
# 10000001 = Maximum temperature                              Float / Integer
# 10000010 = Fan trigger temperature                          Float / Integer
# 10000011 = Minimum cell voltage                             Float / Integer
# 10000100 = Maximum cell voltage                             Float / Integer
# 10000101 = Target cell voltage                              Float / Integer
# 10000110 = Maximum cell voltage difference (for balancing)  Float / Integer
# 10000111 = Time for charge/balance between measurements     Integer
# 10001000 = Verbosity                                        Boolean

# Reading:
# Bits     | Description                  | Value type     | # of bytes
# 00000001 = Total battery voltage          Float            7
# 00000010 = Battery capacity (percentage)  Integer          2
# 00000011 = Mean cell voltage              Float            7
# 00000100 = All cell voltages              List of floats   160
# 00000101 = Board temperatures             List of floats   30
# 00000110 = Status (Error or not)          Integer          1

import board
import busio
import microcontroller
import time
from digitalio import Direction
import adafruit_dotstar as dotstar

class BMS:
    def __init__(self, ADS1248, mcpArr, tmpArr, buzzer, relay, fan):
        self.mode = 0 # 0 = idle, 1 = chg/dschg/storage, 2 = shutdown
        BMS.ADS1248 = ADS1248
        ADS1248.wakeupAll()
        ADS1248.wregAll(2,[0x40,0x03])
        print("[INFO] Calibrating ADCs.")
        ADS1248.selfOffsetAll()
        self.cellCount = 20
        self.drain = [0]*24

        self.mcpArr = mcpArr
        for mcp in self.mcpArr:
            mcp.iodir = 0x00
            mcp.gpio = 0x00

        self.uart = busio.UART(board.TX, board.RX, baudrate=9600, timeout=.1, receiver_buffer_size=16)
        self.uart.write(bytes([127]))

        self.tmpArr = tmpArr
        self.temps = [0]*len(tmpArr)
        self.maxTemp = 80
        self.fanTrigger = 50
        self.fan = fan

        self.buz = buzzer
        self.relay = relay

        self.cellPos = [18, 15, 12, 6, 3, 9, 0, 7, 16, 13, 10, 19, 1, 4, 20, 17, 11, 5, 2, 14]
        self.cells = [0]*self.cellCount
        self.minVoltage = 3.4
        self.maxVoltage = 4.25
        self.targetVoltage = 3.85
        self.dV = .01
        self.balTime = 32
        self.testBalCount = 0
        self.error = False

        self.dot = dotstar.DotStar(board.APA102_SCK, board.APA102_MOSI, 1, brightness=0.5)

        # self.log = open("battVoltages.log", "w")

        self.verbose = False

        if microcontroller.nvm[0] == 1:
            print("[ALERT] A measurement error or severe battery error occured during previous operation.")
            microcontroller.nvm[0] = 0

        self.getCells()
        self.lastCells = list(self.cells)

    def getCells(self):
        if self.verbose:
            print("[INFO] Checking cells...")
        cellRead = BMS.ADS1248.fetchAll(3,[0,1,2,4,5,6,7])
        for i in range(20):
            self.cells[i] = cellRead[self.cellPos[i]]
        # self.log.write(self.cells)
        self.battVoltage = sum(self.cells)
        self.capacity = min(max(round((100/(84-68))*(self.battVoltage-68)),0),100)
        self.meanVoltage = sum(self.cells)/self.cellCount
        self.dot.fill((int((-255/100)*self.capacity+255),int((255/100)*self.capacity),0))

    def getTemps(self):
        self.temps = [None]*len(self.tmpArr)
        for i in range(len(self.tmpArr)):
            self.temps[i] = (150/1.5)*((self.tmpArr[i].value*3.3/65536)-.5)
        self.temps.append(microcontroller.cpu.temperature)
        self.temps = [round(i,2) for i in self.temps]
        if self.verbose:
            print("[INFO] Board temperatures:", [str(i)+"C" for i in self.temps])
        # duty = (65535/30)*(max(self.temps)-30)
        # self.fan.duty_cycle = 0 if duty < 0 else 65535 if duty > 65535 else duty
        if max(self.temps) > self.fanTrigger:
            self.fan.value = True
        else:
            self.fan.value = False
        if max(self.temps) > self.maxTemp + 20:
            print("[ALERT] Thermal shutdown.")
            self.buz.value = True
            self.mode = 2
            return True
        else:
            return False

    def sendIO(self):
        for i in range(len(self.mcpArr)):
            send = 0
            for j in range(len(self.drain[8*i:8*(i+1)])):
                if self.drain[8*i:8*(i+1)][j] == 1:
                    send += 2**j
            self.mcpArr[i].gpio = send

    def balance(self):
        # Target voltage is within min and max voltage
        if self.targetVoltage < self.minVoltage or self.targetVoltage > self.maxVoltage:
            print("[ALERT] Target voltage exceeds acceptable voltage range, terminating charge/balance cycle.")
            self.buz.value = True
            self.relay.value = False
            self.mode = 0
            self.drain = [0]*self.cellCount
            self.sendIO()
            time.sleep(1)
            return
        # Lowest cell is below minimum voltage or avg voltage is below target voltage
        if (self.minCell < self.minVoltage or self.meanVoltage < self.targetVoltage - self.dV/2) and self.maxCell < self.maxVoltage-.05:
            self.charging = True
            if self.verbose:
                print("[INFO] Mean cell voltage is {0}v less than target cell voltage of {1}v.".format(self.targetVoltage-self.meanVoltage, self.targetVoltage))
            print("[INFO] Charging...")
            self.relay.value = True
        else:
            self.charging = False
        # If cells are not balanced or are above target voltage
        if (self.maxCell - self.minCell > self.dV or self.meanVoltage > self.targetVoltage + self.dV/2) and self.minCell > self.minVoltage:
            self.balancing = True
            self.testBalCount = 0
            if self.maxCell - self.minCell > self.dV: # If cells are not balanced set target to minimum cell
                target = self.minCell + self.dV/2
                if self.verbose:
                    print("[INFO] Cell voltage range is {0}v more than {1}v.".format(self.maxCell-self.minCell-self.dV,self.dV))
                print("[INFO] Balancing...")
            elif self.meanVoltage > self.targetVoltage: # Elif cells are above target voltage, set target to target voltage
                target = self.targetVoltage
                if self.verbose:
                    print("[INFO] Mean cell voltage is {0}v more than target cell voltage of {1}v.".format(self.meanVoltage-self.targetVoltage, self.targetVoltage))
                print("[INFO] Discharging...")

            toDschg = []
            for i in range(self.cellCount):
                if self.cells[i] > target:
                    toDschg.append(i)
                else: self.drain[i] = 0
            toDschg = sorted(toDschg,key=lambda x: self.cells[x],reverse=True)
            # print([self.cells[toDschg[i]] for i in range(len(toDschg))])
            # print(toDschg)
            if self.cellCount//2 >= len(toDschg):
                count = len(toDschg)
            else:
                count = int(self.cellCount//2)
            for i in range(count):
                self.drain[toDschg[i]] = 1
            # Now that cells have been selected for discharging, send drain array to IO expanders
            self.sendIO()
        else:
            self.balancing = False

        if not self.balancing and not self.charging:
            if self.testBalCount < 4:
                print("[INFO] Confirming successful charge/balance [{}]...".format(self.testBalCount))
                self.testBalCount += 1
                time.sleep(8)
                return
            else:
                print("\n[INFO] Charge/balance complete!\n")
                print("[INFO] All cells:",self.cells)
                for i in range(2):
                    self.buz.value = True
                    time.sleep(.1)
                    self.buz.value = False
                    time.sleep(.1)
                print("[INFO] Switching to idle.")
                self.mode = 0
                self.testBalCount = 0
        else:
            # Monitor temperature
            for i in range(self.balTime//8):
                self.getTemps()
                if max(self.temps) > self.maxTemp:
                    print("[INFO] Temperature exceeded maximum permitted temperature while balancing.")
                    break
                time.sleep(8)
            # Clear drain and disconnect charger for next update
            self.drain = [0]*self.cellCount
            self.relay.value = False

    def update(self):
        # All the stuff
        if self.mode == 0 or self.mode == 1:
            self.sendIO()
            if self.mode == 1:
                if self.verbose:
                    print("[INFO] Allowing cells to settle...")
                time.sleep(5)
            for i in range(4):
                self.getCells()
                dCells = []
                for i in range(self.cellCount):
                    dCells.append(self.cells[i]-self.lastCells[i])
                if self.mode == 1:
                    if max(dCells) > .05 or min(dCells) < -.05:
                        if self.verbose:
                            print("[INFO] Measure error, trying again...")
                        self.error = True
                    else:
                        self.error = False
                        break
                else:
                    break
            if self.error:
                self.buz.value = True
                microcontroller.nvm[0] = 1
                self.mode = 2
                print("[ALERT] Cell voltage changed rapidly between measurements.\
                       \n\t This could be due to a faulty measurement, or a severe battery issue.\
                       \n\t To avoid possible damage the BMS will shut down.")

            if self.verbose:
                print("[INFO] Mean change in voltage per cell:",sum(dCells)/self.cellCount)

            self.lastCells = list(self.cells)
            if self.verbose:
                # print("[INFO] All cell voltages:\n",self.cells)
                print("[INFO] Battery voltage: {}v".format(self.battVoltage))
                print("[INFO] Battery capacity: {}%".format(self.capacity))
            if self.mode == 2:
                return
            self.buz.value = False
            for i in range(self.cellCount):
                if self.cells[i] > self.maxVoltage:
                    print("\n[ALERT] Cell_{0} is above maximum voltage of {1} at {2}!\n".format(i,self.maxVoltage, self.cells[i]))
                    self.mode = 2
                elif self.cells[i] < self.minVoltage:
                    print("\n[ALERT] Cell_{0} is below minimum voltage of {1} at {2}!\n".format(i,self.minVoltage, self.cells[i]))
                    self.mode = 2
            if self.mode == 2:
                self.buz.value = True
                time.sleep(1)
                self.buz.value = False
                return

            self.minCell = min(self.cells)
            self.maxCell = max(self.cells)
            self.minCellIndex = self.cells.index(self.minCell)
            self.maxCellindex = self.cells.index(self.maxCell)
            if self.verbose:
                print("[INFO] Minimum cell is Cell_{0} with voltage of {1}v.".format(self.minCellIndex,self.minCell))
                print("[INFO] Maximum cell is Cell_{0} with voltage of {1}v.".format(self.maxCellindex,self.maxCell))
                print("[INFO] Mean cell voltage: {}v".format(self.meanVoltage))
                print("[INFO] Cell voltage range: {}v".format(self.maxCell-self.minCell))

            if self.getTemps():
                return

        if self.mode == 1:
            if max(self.temps) < self.maxTemp:
                status = self.balance()
            else:
                print("[INFO] Unable to charge/balance due to high temperatures.")

        elif self.mode == 2:
            self.buz.value = False
            self.relay.value = False
            self.drain = [0]*self.cellCount
            self.sendIO()
            BMS.ADS1248.sleepAll()

        # Sercom
        recv = self.uart.read(1)
        if recv != None:
            command = recv[0]
            print("[UART] Received command:",command)
            try:
                if command >= 128: # Writing information to BMS
                    data = ''.join([chr(i) for i in self.uart.read()])
                    print("[UART] Received data:",data)
                    if command == 128: # Mode
                        if data == "0":
                            self.mode = 0
                        elif data == "1":
                            self.mode = 1
                        elif data == "2":
                            self.mode = 2
                        if self.verbose:
                            print("[INFO] Changed mode to",self.mode)
                    elif command == 133: # Target cell voltage
                        self.targetVoltage = float(data)
                        if self.verbose:
                            print("[INFO] Set target voltage to",float(data))
                    elif command == 129: # Maximum temperature
                        self.maxTemp = float(data)
                        if self.verbose:
                            print("[INFO] Set maximum temperature to",float(data))
                    elif command == 130: # Fan trigger temperature
                        self.fanTrigger = float(data)
                        if self.verbose:
                            print("[INFO] Set fan trigger temperature to",float(data))
                    elif command == 131: # Minimum cell voltage
                        self.minVoltage = float(data)
                        if self.verbose:
                            print("[INFO] Set minimum cell voltage to",float(data))
                    elif command == 132: # Maximum cell voltage
                        self.maxVoltage = float(data)
                        if self.verbose:
                            print("[INFO] Set maximum cell voltage to",float(data))
                    elif command == 134: # Maximum cell voltage difference (for balancing)
                        self.dV = float(data)
                        if self.verbose:
                            print("[INFO] Set maximum cell voltage difference",float(data))
                    elif command == 135: # Time for charge/balance between measurements
                        self.balTime = int(data)
                        if self.verbose:
                            print("[INFO] Set charge/balance time",int(data))
                    elif command == 136: # Verbosity
                        self.verbose = bool(data)
                        if self.verbose:
                            print("[INFO] Set verbosity to",bool(data))


                elif command < 128: # Requesting information from BMS
                    if command == 1: # Battery voltage | 7 bytes
                        self.uart.write(bytes(str(self.battVoltage),'utf-8'))
                        self.uart.write(bytes(''.join(["0" for i in range(7-len(str(self.battVoltage)))]), 'utf-8'))
                        if self.verbose:
                            print("[UART] Sent battery voltage.")
                    elif command == 2: # Battery capacity | 2 bytes
                        self.uart.write(bytes(str(self.capacity),'utf-8'))
                        if self.verbose:
                            print("[UART] Sent battery capacity.")
                    elif command == 3: # Mean cell voltage | 7 bytes
                        self.uart.write(bytes(str(self.meanVoltage),'utf-8'))
                        self.uart.write(bytes(''.join(["0" for i in range(7-len(str(self.meanVoltage)))]), 'utf-8'))
                        if self.verbose:
                            print("[UART] Sent mean cell voltage.")
                    elif command == 4: # All cell voltages | 160 bytes
                        for cell in self.cells:
                            self.uart.write(bytes(str(cell),'utf-8'))
                            self.uart.write(bytes(''.join(["0" for i in range(7-len(str(cell)))]), 'utf-8')+bytes("\n", 'utf-8'))
                        if self.verbose:
                            print("[UART] Sent all cell voltages.")
                    elif command == 5: # Temperatures | 30 bytes
                        for temp in self.temps:
                            self.uart.write(bytes(str(temp),'utf-8'))
                            self.uart.write(bytes(''.join(["0" for i in range(5-len(str(temp)))]), 'utf-8')+bytes("\n", 'utf-8'))
                        if self.verbose:
                            print("[UART] Sent all temperatures.")
                    elif command == 6: # Status | 1 byte
                        if self.mode == 2:
                            self.uart.write(bytes("1", 'utf-8'))
                        else:
                            self.uart.write(bytes("0", 'utf-8'))
                        if self.verbose:
                            print("[UART] Sent current status.")
            except ValueError:
                self.uart.write(bytes(255))
                if self.verbose:
                    print("[ALERT] UART command was formatted incorrectly. Discarding.")
