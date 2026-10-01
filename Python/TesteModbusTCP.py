from pyModbusTCP.client import ModbusClient

client = ModbusClient(host='192.168.15.120', port=502)

if client.open():

    # ligar M0
    client.write_single_coil(0, False)

    print("M0 acionado")

    client.close()
