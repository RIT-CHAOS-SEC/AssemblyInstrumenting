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
	dir_call_instrs	      = ['bl']
	conditional_br_instrs = ['beq','bne','bhs','blo','bhi','bls','bgt','blt','bge','ble','bcs','bcc','bmi','bpl','bvs','bvc']

	conditional_dests = []
	forward_cond_dests = []

	write_ops = ['mov', 'movs', 'ldr']

	trampoline = "SECURE"
	trampoline_cond_br = "SECURE_log_cond_br"
	trampoline_ret = "SECURE_log_ret"
	# trampoline_call = "SECURE_log_call"
	# trampoline_indr_jmp = "SECURE_log_indr_jmp"
	trampoline_indr_fwd = "SECURE_log_indr_fwd"

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

		if len(parsed) == 1 and '.L' in parsed[0]:

			label = parsed[0].replace(':', '')
			try:
				bt = max(int(label.split('L')[1]), bt)
				# print(label)
				# print(f"bt = {bt}")
			except ValueError:
				# some data labels are LC#
				# we can ignore
				debug_print(f"Ignoring label processing of {label}", file=outfile)

			print(x, file=outfile)
			if label in conditional_dests:
				# print(label)
				forward_cond_dests.append(label)

		# Assembly line is an instruction
		elif len(parsed) > 2:

			inst = parsed[1]
			args = parsed[2]
			
			detect_cond_branch = (inst in conditional_br_instrs)
			detect_ret_inst = (inst in ret_instrs)
			detect_ret_via_pop = (inst == 'pop') and ('pc' in args)
			detect_call_inst = (inst in call_instrs)

			detect_indr_jump = (inst in write_ops) and ('pc' in args)

			if detect_call_inst:
				case = 2
				debug_print("------ instrumenting call ("+inst+" "+args+")", file=outfile)
				if inst == 'bl': # direct call, use ldr
					case = 3
					print("\tldr\tr10, ="+args, file=outfile)
				elif inst == 'blx': # indirect call, can use mov
					case = 4
					print("\tmov\tr10, "+args, file=outfile)
				print("\tbl\t"+trampoline_indr_fwd, file=outfile)

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

			elif detect_indr_jump:
				debug_print("------ instrumenting indr jump ("+inst+") "+args, file=outfile)
				# replace r10 with 
				print((f"\t{inst}\t{args}").replace("pc", "r10"), file=outfile)
				print("\tb\t"+trampoline_indr_fwd, file=outfile)
			else:
				case = 7
				print(x, file=outfile)
		else:
			case = 8
			print(x, file=outfile)

	outfile.close()

	infile = open(outfile_name, "r")
	lines = infile.readlines()
	# print(f'FIRST LINE: {lines[0]}')
	infile.close()

	lines = [x.replace('\n','') for x in lines]
	# print(f'FIRST LINE: {lines[0]}')
	# second pass -- instrument cond.branch taken
	
	ready = True
	forward_conds = {}
	bt += 1
	for i in range(0, len(forward_cond_dests)):
		forward_conds[forward_cond_dests[i]] = bt
		bt += 1

	inst = ''
	# print(f"bt = {bt}")
	i = 0
	new_lines = []
	debug_continue = True
	while i < len(lines) and debug_continue:
		x = lines[i]
		parsed = x.split('\t')
		if ".file" in x or 'nop' in x:
			inst = ''
		elif len(parsed) > 2:
			inst = ''
			if (parsed[1] in conditional_br_instrs) and parsed[2] in forward_cond_dests:
				inst = parsed[1]
				args = parsed[2]
				new_lines.append(x.replace(args, '.L'+str(forward_conds[args])))
				# print(x.replace(args, '.L'+str(forward_conds[args])), file=outfile)
			else:
				# print(x, file=outfile)	
				new_lines.append(x)
		elif len(parsed) == 1:
			inst = ''
			label = parsed[0].split(":")[0]
			if label in forward_cond_dests:
				prev = lines[i-1].split('\t')
				# dont need to add branch if prev. is a dir branch
				# print(f'prev: {prev}')
				if len(prev) > 2:
					inst = prev[1]
					if 'b' != inst:
						# print(f'inst: {inst}')
						# print(f'\tb\t{label}', file=outfile)
						new_lines.append(f'\tb\t{label}')
						# a = input()
				# print(f'.L{forward_conds[label]}:', file=outfile)
				# print(f'\tbl\t{trampoline_cond_br}_taken', file=outfile)
				# print(x, file=outfile)
				new_lines.append(f'.L{forward_conds[label]}:')
				new_lines.append(f'\tbl\t{trampoline_cond_br}_taken')
				new_lines.append(x)
				conditional_dests.remove(label)
			elif label in conditional_dests:

				# is a loop
				label = parsed[0].split(":")[0]
				j = i+1
				empty_Loop = True
				while label not in lines[j]:
					for inst in call_instrs+dir_call_instrs:
						if inst in lines[j]:
							empty_Loop = False
					for inst in conditional_br_instrs:
						if inst in lines[j] and label not in lines[j]:
							empty_Loop = False
					j += 1
				# print(f"Empty Loop: {empty_Loop}")
				# print(f'Label line: {lines[j]}')
				# print(f'Cond line: {lines[j-1]}')
				# a = input()
			
				if empty_Loop:
					_, inst, args = lines[j-1].split('\t')
					comp_reg, comp_base = args.split(', ')
					
					if label == '.L45':
						print(f'{comp_reg, comp_base}')
					# traverse back until find where comp_reg is first written
					# print(f'comp_reg: {comp_reg}')
					loop_end_l = j
					while not ('mov' in lines[j] and comp_reg in lines[j]) and not ('mov' in lines[j] and comp_base in lines[j]) and not ('push' in lines[j]):
						j -= 1
					loop_start_l = j
					# if 'push' in lines[j]:
					if True:
						# is modified or input, so can't optimize
						empty_Loop = False

					else:
						# # assumed format of (comp_reg, comp_base), but was actually (comp_base, comp_reg)
						# if ('mov' in lines[j] and comp_base in lines[j]):
						# 	tmp = comp_reg
						# 	comp_reg = comp_base
						# 	comp_base = tmp 
						# print(f'Found in lines at {j}: {lines[j]}')
						dec = 0
						while not ('mov' in new_lines[dec] and comp_reg in new_lines[dec]) and not ('mov' in new_lines[dec] and comp_base in new_lines[dec]) and not ('push' in lines[j]):
							dec -=1
						dec = len(new_lines)+dec
						# print(f'len(new_lines): {len(new_lines)}')
						# print(f'dec: {dec}')
						
						# print(f'Found in new_lines at {dec}: {new_lines[dec]}')
						if '#' in comp_base:
							print(dec)
							# a = input()
							## insert instrumentation to log the br-taken address and loop condition
							inserted = []
							inserted.append(f'\tadr\tr10, {label}')
							inserted.append(f'\tmov\tr11, {comp_base}')
							inserted.append(f'\tbl\t{trampoline}_log_loop_cond')
							new_lines = new_lines[:dec] + inserted + new_lines[dec:] + [x]
							# print('GOT HERE')
							# if label == '.L45':
							# 	print(len(new_lines))
							# 	debug_continue = False
						else:
							## loop base is a reg. check if it is also modified in the loop
							notModified = True
							load_args = None
							idxs = list(range(loop_start_l+1, loop_end_l-1))[::-1]
							print(idxs)
							for j in idxs:
								print(lines[j])
								if (comp_base in lines[j] and ('str' not in lines[j] and 'ldr' not in lines[j])):
									notModified = False
									print(f'modified here')
								if (comp_base in lines[j] and 'ldr' in lines[j]):
									_, _, load_args = lines[j].split('\t')
									load_args = load_args.replace(comp_base+',', '')
									break
									# print(f"load_args: {load_args}")
									# a = input()
									# print(lines[j])
							print(f'{comp_base} not Modified? --> {notModified}')	
							print(f'load_args: {load_args}')
							if notModified:
								# if not modified, log the initial value
								## add the optimized version
								inserted = []
								inserted.append(f'\tadr\tr10, {label}')
								if load_args is not None:
									inserted.append(f'\tldr\tr11, {load_args}')
								else:
									inserted.append(f'\tmov\tr11, {comp_base}')
								inserted.append(f'\tbl\t{trampoline}_log_loop_cond')
								print('inserted: ')
								for isr in inserted:
									print(isr)
								print(f'dec: {dec}')
								new_lines = new_lines[:dec] + inserted + new_lines[dec:] + [x]
							else:
								# if modified, log normally
								new_lines.append(x)
								new_lines.append("\tbl\t"+trampoline_cond_br+'_taken')
							# a = input()
			
				if not empty_Loop:
					# there is internal branching, so instrument in typical manner
					debug_print("------ instrumenting cond branch dest ("+x+")", file=outfile)
					# print(x, file=outfile)
					# print("\tbl\t"+trampoline_cond_br+'_taken', file=outfile)
					new_lines.append(x)
					new_lines.append("\tbl\t"+trampoline_cond_br+'_taken')

			else:
				new_lines.append(x)
				# print(x, file=outfile)	
		else:
			new_lines.append(x)
			# print(x, file=outfile)	
		i += 1
	outfile = open(outfile_name, "w")
	print(len(new_lines))
	for line in new_lines:
	    outfile.write(line + "\n")
	outfile.close()

	print("------------------")
	print("Conditional br destinations: "+str(conditional_dests))
	print("Forward cond. br destinations: "+str(forward_cond_dests))
	print("------------------")
	#'''

if __name__ == '__main__':
	args = arg_parser()

	instrment_asm(args.dir, args.infile, args.outfile)