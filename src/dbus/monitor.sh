#Per ricevere eventi dbus da remoto:
dbus-monitor --address "tcp:host=192.168.1.66,port=12345" "type='signal',sender='org.mydrone.MyPIDControllerService',interface='org.mydrone.MyPIDControllerService'"

