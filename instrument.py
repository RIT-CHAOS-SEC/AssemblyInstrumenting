import sys
import argparse

def arg_parser():
    '''
    Parse the arguments of the program
    Return:
        object containing the arguments
    '''
    parser = argparse.ArgumentParser()
    parser.add_argument('--infile', type=str, required=True,
                        help='input assembly file name (.s)')
    parser.add_argument('--dir', type=str, default='./',
                        help='directory of input/output files')
    parser.add_argument('--outfile', type=str, default='instrumented_app',
                        help='output file name (.s)')
    
    args = parser.parse_args()
    return args

def debug_print(string, file):
	if file.name == "debug.s":
		print(string, file=file)

def instrment_asm(directory, input_file, output_file):
	# directory = "./example_tz/NonSecure/Debug/Assembly/"
	# file = "application.s"
	
	infile_name = directory+input_file

	outfile_name = directory+output_file
	# outfile_name = "debug.s"

	app_entry_func = 'test_application'

	ret_instrs    		  = ['bx', 'bxeq']
	call_instrs			  = ['blx'] # bl treated as direct branches
	conditional_br_instrs = ['beq','bne','bhs','blo','bhi','bls','bgt','blt','bge','ble','bcs','bcc','bmi','bpl','bvs','bvc']

	conditional_dests = []
	trampoline = "SECURE"
	trampoline_cond_br = "SECURE_log_cond_br"
	trampoline_call = "SECURE_log_call"
	trampoline_ret = "SECURE_log_ret"

	offset = 0

	infile = open(infile_name, "r")
	lines = infile.readlines()
	infile.close()
	
	outfile = open(outfile_name, "w")

	lines = [x.replace('\n','') for x in lines if x != '\n']

	last_push = ""
	i = 0
	#preliminary pass -- always push lr, change returns to pop
	while i < len(lines):
		x = lines[i]
		i += 1
		parsed = x.split('\t')
		if "test_application" in x:
			x = x.replace("test_application", "application")

		if "application.c" in x:
			continue

		# if '@' in x or ".file" in x:
		# 	continue 

		if trampoline in x:
			print(x, file=outfile)
			case = 1

		# Assembly line is an instruction
		elif len(parsed) > 2:
			inst = parsed[1]
			args = parsed[2]

			if inst == 'push' and 'lr' not in args:
				print(x.replace('}', ', lr}'), file=outfile)

			elif inst == 'ldr' and 'sp' in args and 'r7' in args:
				print('\tpop\t{r7, pc}', file=outfile)			
				i += 1
			elif inst == 'pop' and 'pc' not in args:
				print(x.replace('}', ', pc}'), file=outfile)				
			else:
				print(x, file=outfile)
		else:
			print(x, file=outfile)
	
	outfile.close()

	infile = open(outfile_name, "r")
	lines = infile.readlines()
	infile.close()

	#'''
	outfile = open(outfile_name, "w")
	lines = [x.replace('\n','') for x in lines if x != '\n']
	i = 0
	case = 0
	# first pass -- instrument calls and returns, and cond.branch not taken
	
	forward_cond_dests = []
	bt = 0
	while i < len(lines):
		x = lines[i]
		i += 1
		parsed = x.split('\t')

		if "application.c" in x:
			continue

		# if '@' in x or ".file" in x:
		# 	continue 

		if trampoline in x:
			print(x, file=outfile)
			case = 1

		if len(parsed) == 1 and '.' in parsed[0]:

			label = parsed[0].replace(':', '')
			try:
				bt = max(int(label.split('L')[1]), bt)
			except ValueError:
				# some data labels are LC#
				# we can ignore
				debug_print(f"Ignoring label processing of {label}", file=outfile)
				
			print(x, file=outfile)
			if label in conditional_dests:
				forward_cond_dests.append(label)

		# Assembly line is an instruction
		elif len(parsed) > 2:

			inst = parsed[1]
			args = parsed[2]
			
			detect_cond_branch = (inst in conditional_br_instrs)
			detect_ret_inst = (inst in ret_instrs)
			detect_ret_via_pop = (inst == 'pop') and ('pc' in args)
			detect_call_inst = (inst in call_instrs)

			if detect_call_inst:
				case = 2
				debug_print("------ instrumenting call ("+inst+" "+args+")", file=outfile)
				if inst == 'bl': # direct call, use ldr
					case = 3
					print("\tldr\tr10, ="+args, file=outfile)
				elif inst == 'blx': # indirect call, can use mov
					case = 4
					print("\tmov\tr10, "+args, file=outfile)
				print("\tbl\t"+trampoline_call, file=outfile)

			elif detect_cond_branch:
				case = 5
				debug_print("------ instrument cond branch not taken ("+inst+") "+args, file=outfile)
				conditional_dests.append(args)
				print(x, file=outfile)
				print("\tbl\t"+trampoline_cond_br+'_not_taken', file=outfile)

			# # 	debug_print("------ ", file=outfile)
			elif detect_ret_via_pop:
				case = 6
				#Instrument before return
				debug_print("------ instrumenting ret via pop ("+inst+") "+args, file=outfile)
				print(("\tpop\t"+args).replace("pc", "lr"), file=outfile)
				print("\tb\t"+trampoline_ret, file=outfile)
				
			# 	debug_print("------ ", file=outfile)
			elif detect_ret_inst:
				debug_print("------ instrumenting ret via bx ("+inst+") "+args, file=outfile)
				print("\tb\t"+trampoline_ret, file=outfile)

			else:
				case = 7
				print(x, file=outfile)
		else:
			case = 8
			print(x, file=outfile)

	print("------------------")
	print("Conditional br destinations: "+str(conditional_dests))
	print("Forward cond. br destinations: "+str(forward_cond_dests))
	print("------------------")

	outfile.close()

	infile = open(outfile_name, "r")
	lines = infile.readlines()
	infile.close()

	outfile = open(outfile_name, "w")

	lines = [x.replace('\n','') for x in lines]
	# second pass -- instrument cond.branch taken
	
	ready = True
	forward_conds = {}
	bt += 1
	for i in range(0, len(forward_cond_dests)):
		forward_conds[forward_cond_dests[i]] = bt
		bt += 1

	inst = ''
	i = 0
	while i < len(lines):
		x = lines[i]
		parsed = x.split('\t')
		if ".file" in x or 'nop' in x:
			inst = ''
		elif len(parsed) > 2:
			inst = ''
			if (parsed[1] in conditional_br_instrs) and parsed[2] in forward_cond_dests:
				inst = parsed[1]
				args = parsed[2]
				print(x.replace(args, '.L'+str(forward_conds[args])), file=outfile)
			else:
				print(x, file=outfile)	
		elif len(parsed) == 1:
			inst = ''
			label = parsed[0].split(":")[0]
			if label in forward_cond_dests:
				prev = lines[i-1].split('\t')
				# dont need to add branch if prev. is a dir branch
				if len(prev) > 2:
					inst = prev[1]
					if 'b' != inst:
						print(f'\tb\t{label}', file=outfile)
						# a = input()
				print(f'.L{forward_conds[label]}:', file=outfile)
				print(f'\tbl\t{trampoline_cond_br}_taken', file=outfile)
				print(x, file=outfile)
			elif label in conditional_dests:
				label = parsed[0].split(":")[0]
				debug_print("------ instrumenting cond branch dest ("+x+")", file=outfile)
				print(x, file=outfile)
				print("\tbl\t"+trampoline_cond_br+'_taken', file=outfile)
			else:
				print(x, file=outfile)	
		else:
			print(x, file=outfile)
		i += 1
	outfile.close()
	#'''

if __name__ == '__main__':
	args = arg_parser()

	instrment_asm(args.dir, args.infile, args.outfile)