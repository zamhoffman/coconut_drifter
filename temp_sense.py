from time import sleep, monotonic
import board
import busio
import digitalio
import circuitpython_AD7124 as NHB_AD7124
filterSelectBits = 4  # Fast

#table from https://www.italcoppie.com/wp-content/uploads/2022/08/Pt1000-Resistance-Chart-A4.pdf
#from -10 to 110 C
pt1000_table = [960.9, 964.8, 968.7, 972.6, 976.5, 980.4, 984.4, 988.3, 992.2, 996.1,
                1000, 1003.9, 1007.8, 1011.7, 1015.6, 1019.5, 1023.4, 1027.3, 1031.2, 1035.1,
                1039.0, 1042.9, 1046.8, 1050.7, 1054.6, 1058.5, 1062.4, 1066.3, 1070.2, 1074.0,
                1077.9, 1081.8, 1085.7, 1089.6, 1093.5, 1097.3, 1101.2, 1105.1, 1109.0, 1112.9,
                1116.7, 1120.6, 1124.5, 1128.3, 1132.2, 1136.1, 1140.0, 1143.8, 1147.7, 1151.5,
                1155.4, 1159.3, 1163.1, 1167.0, 1170.8, 1174.7, 1178.6, 1182.4, 1186.3, 1190.1,
                1194.0, 1197.8, 1201.7, 1205.5, 1209.4, 1213.2, 1217.1, 1220.9, 1224.7, 1228.6,
                1232.4, 1236.3, 1240.1, 1243.9, 1247.8, 1251.6, 1255.4, 1259.3, 1263.1, 1266.9,
                1270.8, 1274.6, 1278.4, 1282.2, 1286.1, 1289.9, 1293.7, 1297.5, 1301.3, 1305.2,
                1309.0, 1312.8, 1316.6, 1320.4, 1324.2, 1328.0, 1331.8, 1335.7, 1339.5, 1343.3,
                1347.1, 1350.9, 1354.7, 1358.5, 1362.3, 1366.1, 1369.9, 1373.7, 1377.5, 1381.3,
                1385.1, 1388.8, 1392.6, 1396.4, 1400.2, 1404.0, 1407.8, 1411.6, 1415.4, 1419.1,
                1422.9]

adc = NHB_AD7124.Ad7124()

samples = 64

def get_voltages():
    print("Trying to read ID register...")
    ID = adc.get_ID()
    print(f"ADC chip ID: {hex(ID)}")


    # Configure the ADC in full power mode
    adc.set_adc_control(
        NHB_AD7124.AD7124_OpMode_SingleConv, NHB_AD7124.AD7124_FullPower, True
    )

    # Configure Setup 0 for load cells:
    # - Use the external reference tied to the excitation voltage (2.5V reg)
    # - Set a gain of 128 for a bipolar measurement of +/- 19.53 mv
    adc.setup[0].set_config(NHB_AD7124.AD7124_Ref_ExtRef1, NHB_AD7124.AD7124_Gain_1, True)

    # Set filter type and data rate select bits (defined above)

    adc.setup[0].set_filter(NHB_AD7124.AD7124_Filter_SINC3, filterSelectBits)

    # Set channel 0 to use pins AIN0(+)/AIN1(-)
    adc.set_channel(0, 0, NHB_AD7124.AD7124_Input_AIN0, NHB_AD7124.AD7124_Input_AIN1, True)
    adc.set_channel(1, 0, NHB_AD7124.AD7124_Input_AIN2, NHB_AD7124.AD7124_Input_AIN3, True)


    print("Turning ExV on")
    adc.setPWRSW(True)
    sleep(0.2)


    print("Now try to get some readings...")
    reading_RTD = 0
    reading_REF = 0

    for i in range(samples):
        reading_RTD += adc.read_volts(0)
        reading_REF += adc.read_volts(1)

    reading_RTD = reading_RTD/samples
    reading_REF = reading_REF/samples

    print(f"RTD Voltage = {reading_RTD:.4f}", end=",")
    print(f"REF Voltage = {reading_REF:.4f}", end=",")

    return reading_RTD, reading_REF

def get_temp_measurement(REF_val):
    volt_RTD, volt_REF = get_voltages()

    volt_RTD = abs(volt_RTD)
    volt_REF = abs(volt_REF)

    print(volt_RTD, volt_REF)

    curr_RTD = volt_REF/REF_val

    res_RTD = volt_RTD/curr_RTD

    print("res val: ", res_RTD)

    val_index = 0
    for elem in pt1000_table:
        if elem >= res_RTD:
            val_index = pt1000_table.index(elem)
            print("elem :", elem)
            break
    print("index: ", val_index)

    #linear interpolate (does not account for edgecases)
    v1 = pt1000_table[val_index-1]
    v2 = pt1000_table[val_index]
    temp = -10 + val_index + (0.1/(v2-v1))*(res_RTD - v1)

    return temp




