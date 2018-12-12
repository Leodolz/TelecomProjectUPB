import sys
import ttn
import argparse
import math
from influxdb import InfluxDBClient

receivedNodeUplinkMsg = False
nodeMsg = 0
# LPGCurve = {2.3, 0.2, -0.45}
# SmokeCurve = {2.3, 0.53, -0.43}
# COCurve = {1.7, 0.255, -0.67}
valhumo = 0.0
valprop = 0.0
valco = 0.0
valhume = 0.0
valtemp = 0.0
cond = 1
rawValuee1 = 0
rawValuee2 = 0


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


def uplinkCallback(msg, client):

    global receivedNodeUplinkMsg
    global nodeMsg3
    global nodeMsg4
    global rawValuee1
    global rawValuee2
    print("\r\nThis is the payload:")
    print(msg.payload_fields.rawValue1)
    print(type(msg.payload_fields.rawValue1))
    print(msg.payload_fields.rawValue2)
    print(type(msg.payload_fields.rawValue2))
    print(msg.payload_fields.rawValue3)
    print(type(msg.payload_fields.rawValue3))
    print(msg.payload_fields.rawValue4)
    print(type(msg.payload_fields.rawValue4))
    receivedNodeUplinkMsg = True
    rawValuee1 = msg.payload_fields.rawValue1
    rawValuee2 = msg.payload_fields.rawValue2
    nodeMsg3 = msg.payload_fields.rawValue3
    nodeMsg4 = msg.payload_fields.rawValue4
    if cond == 1:
        calibrado = (10 * (4096 - rawValuee1) / rawValuee1) / 9.83
        calibra2 = (10 * (4096 - rawValuee2) / rawValuee2) / 26.333
    else:
        valhumo = ReadMQ(rawValuee1, 2, calibrado)
        valprop = ReadMQ(rawValuee1, 1, calibrado)
        valco = ReadMQ(rawValuee2, 3, calibra2)
    print("\r\nSmoke Readings: ")
    print(type(valhumo))
    print(valhumo)
    print("\r\nPropane Gas Readings: ")
    print(type(valprop))
    print(valprop)
    print("\r\nCO Readings")
    print(type(valco))
    print(valco)
    print("\r\nHumidity Readings: ")
    print(type(valhume))
    print(valhume)
    print("\r\nTemperature Readings: ")
    print(type(valtemp))
    print(valtemp)
    cond = cond + 1


def main():

    global receivedNodeUplinkMsg

    parser = argparse.ArgumentParser(description='TTN cloud and \
                                    database connection')

    parser.add_argument("-d",
                        "--dbaddr",
                        dest='dbaddr',
                        help="database network address",
                        type=str,
                        default='localhost'
                        )

    parser.add_argument("-a",
                        "--appid",
                        dest='appid',
                        help="TTN application id",
                        type=str,
                        default='testapptelecomproject_868'
                        )

    parser.add_argument("-k",
                        "--accessKey",
                        dest='accessKey',
                        help="TTN Application Access Key",
                        type=str,
                        default='ttn-account-v2.2GeA7EbqY7SBG-14tG7ma5m4I17VoQTJ4UZS0ZP1-ZQ'
                        )

    args = parser.parse_args()
    dbAddress = args.dbaddr
    app_id = args.appid
    access_key = args.accessKey
    # ttn application credentials
    # app_id = "testapptelecomproject_868"
    # access_key = "ttn-account-v2.2GeA7EbqY7SBG-14tG7ma5m4I17VoQTJ4UZS0ZP1-ZQ"

    # TTN handlers
    handler = ttn.HandlerClient(app_id, access_key)
    # using mqtt client
    mqtt_client = handler.data()
    # set the callback function for uplink-Msgs
    mqtt_client.set_uplink_callback(uplinkCallback)
    mqtt_client.connect()

    # connect to database
    client = InfluxDBClient(host=dbAddress, port=8086)
    client.switch_database('MySensors')

    while True:
        try:
            if receivedNodeUplinkMsg:
                json_body = [
                    {
                        "measurement": "VarResistorData",
                        "tags": {
                            "Sensor": "varRes1",
                        },
                        "fields": {
                            "rawValue1": rawValuee1,
                            "rawValue2": rawValuee2,
                            "rawValue3": nodeMsg3,
                            "rawValue4": nodeMsg4
                        }
                    }]

                # write to database
                client.write_points(json_body)
                # reset uplink-msg flag
                receivedNodeUplinkMsg = False

        except KeyboardInterrupt:
            mqtt_client.close()
            print()
            print("You Pressed CTRL+C")
            print("Quiting Program - Bye!")
            sys.exit(0)

    # time.sleep(60)
    # mqtt_client.close()


if __name__ == '__main__':
    main()
