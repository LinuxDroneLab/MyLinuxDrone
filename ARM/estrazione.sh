#fare sftp root@192.168.1.65; get /var/log/syslog poi eseguire i comandi sotto
grep 'PR(' syslog > syslog.transitorio
cp syslog.transitorio ../../scilab/syslog.sci
cat /hd/eworkspace/scilab/syslogplot.sci >> /hd/eworkspace/scilab/syslog.sci
#Poi sistemare e ripulire i dati nel file /hd/eworkspace/scilab/syslog.sci

