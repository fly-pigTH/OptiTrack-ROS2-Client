# -*- coding: utf-8 -*-
import time
import binascii
import serial

class ForceSensor(object):
    def __init__(self, ComName):
        self.ser = serial.Serial(ComName, 230400, timeout=10)
        if self.ser.isOpen():
            print("open FS success")
        else:
            print("open FS failed")
    def get_fs_value(self):
        try:
            readed_data = []
            data_to_deal = []
            wrench = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            serialsendcommandhex = b"\x49\xAA\x0D\x0A"
            self.ser.write(serialsendcommandhex)
            # read and push to buffer
            time.sleep(0.02)
            count = self.ser.inWaiting()
            #print("send+++++")
            #print(count)
            if (count > 0):
                data = self.ser.read(count)
                datalist = bytearray(data)
                for i in range(count):
                    readed_data.append(datalist[i])
            # deal the buffer
            #print readed_data
            #print len(readed_data)
            while (len(readed_data) > 11):
                readed_data_len = len(readed_data)
                if ((readed_data_len >= 12) and (readed_data[10] == 13) and (readed_data[11] == 10)):
                    for i in range(0, 12):
                        data_to_deal.append(readed_data.pop(0))  # ////////////////
                    if ((data_to_deal[1] & 0x80) > 0):  # 0x80=1000 0000,>0
                        deal_temp = data_to_deal[1]
                        deal_temp = deal_temp << 4  # *16
                        deal_temp = deal_temp | (data_to_deal[2] >> 4)
                        deal_temp = deal_temp - 4096
                    else:
                        deal_temp = data_to_deal[1]
                        deal_temp = deal_temp << 4
                        deal_temp = deal_temp | (data_to_deal[2] >> 4)
                    wrench[0] = deal_temp * 0.002734375
                    deal_temp = 0
                    if ((data_to_deal[2] & 0x08) > 0):
                        deal_temp = data_to_deal[2] & 0x0f
                        deal_temp = deal_temp << 8
                        deal_temp = deal_temp | data_to_deal[3]
                        deal_temp = deal_temp - 4096
                    else:
                        deal_temp = data_to_deal[2] & 0x0f
                        deal_temp = deal_temp << 8
                        deal_temp = deal_temp | data_to_deal[3]
                    wrench[1] = deal_temp * 0.002734375
                    deal_temp = 0
                    if ((data_to_deal[4] & 0x80) > 0):
                        deal_temp = data_to_deal[4]
                        deal_temp = deal_temp << 4
                        deal_temp = deal_temp | (data_to_deal[5] >> 4)
                        deal_temp = deal_temp - 4096
                    else:
                        deal_temp = data_to_deal[4]
                        deal_temp = deal_temp << 4
                        deal_temp = deal_temp | (data_to_deal[5] >> 4)
                    wrench[2] = deal_temp * 0.002734375
                    deal_temp = 0
                    if ((data_to_deal[5] & 0x08) > 0):
                        deal_temp = data_to_deal[5] & 0x0f
                        deal_temp = deal_temp << 8
                        deal_temp = deal_temp | data_to_deal[6]
                        deal_temp = deal_temp - 4096
                    else:
                        deal_temp = data_to_deal[5] & 0x0f
                        deal_temp = deal_temp << 8
                        deal_temp = deal_temp | data_to_deal[6]
                    wrench[3] = deal_temp * 0.00009765625
                    deal_temp = 0
                    if ((data_to_deal[7] & 0x80) > 0):
                        deal_temp = data_to_deal[7]
                        deal_temp = deal_temp << 4
                        deal_temp = deal_temp | (data_to_deal[8] >> 4)
                        deal_temp = deal_temp - 4096
                    else:
                        deal_temp = data_to_deal[7]
                        deal_temp = deal_temp << 4
                        deal_temp = deal_temp | (data_to_deal[8] >> 4)
                    wrench[4] = deal_temp * 0.00009765625
                    deal_temp = 0
                    if ((data_to_deal[8] & 0x08) > 0):
                        deal_temp = data_to_deal[8] & 0x0f
                        deal_temp = deal_temp << 8
                        deals_temp = deal_temp | data_to_deal[9]
                        deal_temp = deal_temp - 4096
                    else:
                        deal_temp = data_to_deal[8] & 0x0f
                        deal_temp = deal_temp << 8
                        deal_temp = deal_temp | data_to_deal[9]
                    wrench[5] = deal_temp * 0.00009765625
                    for i in range(0, 12):
                        data_to_deal.pop(0)     # NOTE: 这里不是很高效，可以优化
                elif (readed_data_len >= 12):
                    if (readed_data[0] == 0x0a):
                        readed_data.pop(0)
                    else:
                        i = 0
                        while ((i <= 12) and (readed_data[0] != 0x0d) and (readed_data[1] != 0x0a)):
                            readed_data.pop(0)
                            i = i + 1
                        readed_data.pop(0)
                        readed_data.pop(0)
            return wrench   # [2]  # 单位是N

        except Exception as e:
            print("Error:", e)


if __name__ == '__main__':
    # 测试
    fs = ForceSensor("COM12")
    while True:
        res = fs.get_fs_value()
        time.sleep(1)
        print(res)