messages = open("Airmar-maretron-output.txt", "r")
messages = [line.strip().split('*') for line in messages]

print(messages)

def checksum(instr, sum):
	if len(instr.encode('utf-8')) == int(sum, 16):
		return True
	print('Instr: ' + instr + ' failed checksum - val: ' + str(len(instr.encode('utf-8'))) + ', check: ' + str(int(sum, 16)))
	return False

for line, sum in messages:
	checksum(line, sum)