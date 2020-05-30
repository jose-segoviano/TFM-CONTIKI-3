mosquitto_pub -h localhost -t "/position/08" -m "45|45|R"
mosquitto_pub -h localhost -t "/position/09" -m "45|75|R"
mosquitto_pub -h localhost -t "/position/0a" -m "75|45|R"
mosquitto_pub -h localhost -t "/position/0b" -m "75|75|R"
#python udp-server.py