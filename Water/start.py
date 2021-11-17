import paramiko 
host = "192.168.0.24"
user = 'pi'
secret = '1'
port = 22
client = paramiko.SSHClient()
client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
client.connect(hostname=host, username=user, password=secret, port=port)
stdin, stdout, stderr = client.exec_command('python sample_client.py')
print("fdsfsd")
data = stdout.read() + stderr.read()
print(data)
client.close()
