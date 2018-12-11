import math


# LPGCurve = {2.3, 0.2, -0.45}
# SmokeCurve = {2.3, 0.53, -0.43}
# COCurve = {1.7, 0.255, -0.67}
valhumo = 0.0
valprop = 0.0
valco = 0.0
valhume = 0.0
valtemp = 0.0


def GetPercentage(rs_ro_ratio, curve1, curve2, curve3):
    return (pow(10, (((math.log10(rs_ro_ratio) + curve2) / curve3) + curve1)))


def ResistanceCalculation(raw_adc):
    return (10 * (4096 - raw_adc) / raw_adc)


def ReadSensor(raw):
    return ResistanceCalculation(raw)


def GetGasPercentage(rs_ro_ratio, gas_id):
    if gas_id == 0:

        return GetPercentage(rs_ro_ratio, 2.3, 0.2, -0.45)
    elif gas_id == 1:
        return GetPercentage(rs_ro_ratio, 2.3, 0.53, -0.43)
    else:
        return GetPercentage(rs_ro_ratio, 1.7, 0.2555, -0.67)


def ReadMQ(crudo, cual, di):
    if cual == 1:
        return (GetGasPercentage(ReadSensor(crudo) / di, 0))
    elif cual == 2:
        return (GetGasPercentage(ReadSensor(crudo) / di, 1))
    else:
        return (GetGasPercentage(ReadSensor(crudo) / di, 2))


def main():
    print("Comenzando programa\n")
    rawValue1 = 200
    rawValue2 = 200
    temperature = 267
    humidity = 330
    calibrado = (10 * (4096 - rawValue1) / rawValue1) / 9.83
    calibra2 = (10 * (4096 - rawValue2) / rawValue2) / 26.333
    valhumo = ReadMQ(rawValue1, 1, calibrado)
    valprop = ReadMQ(rawValue1, 2, calibrado)
    valco = ReadMQ(rawValue2, 3, calibra2)
    print("Estos son los valores leidos\n")
    print(valhumo)
    print("\n")
    print(valprop)
    print("\n")
    print(valco)
    print(temperature)
    print("\n")
    print(humidity)
    print("\n")


if __name__ == '__main__':
    main()
