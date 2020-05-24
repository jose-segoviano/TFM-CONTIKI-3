mosquitto_pub -h localhost -t "/position/05" -m "1|1|R"
mosquitto_pub -h localhost -t "/position/06" -m "30|10|R"
mosquitto_pub -h localhost -t "/position/07" -m "10|30|R"
python udp-server.py