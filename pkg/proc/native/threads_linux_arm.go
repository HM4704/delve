package native

import (
	"os"
	"debug/elf"
	"encoding/binary"
	"fmt"
	"golang.org/x/arch/arm/armasm"
	"math/bits"
	"syscall"
	"unsafe"
	"bytes"

	sys "golang.org/x/sys/unix"

	"github.com/go-delve/delve/pkg/proc"
	"github.com/go-delve/delve/pkg/proc/linutil"
)

func (t *nativeThread) fpRegisters() ([]proc.Register, []byte, error) {
	var err error
	var arm_fpregs linutil.ARMPtraceFpRegs
	t.dbp.execPtraceFunc(func() { arm_fpregs.Vregs, err = ptraceGetFpRegset(t.ID) })
	fpregs := arm_fpregs.Decode()
	if err != nil {
		err = fmt.Errorf("could not get floating point registers: %v", err.Error())
	}
	return fpregs, arm_fpregs.Vregs, err
}

func (t *nativeThread) restoreRegisters(savedRegs proc.Registers) error {
	sr := savedRegs.(*linutil.ARMRegisters)

	var restoreRegistersErr error
	t.dbp.execPtraceFunc(func() {
		restoreRegistersErr = ptraceSetGRegs(t.ID, sr.Regs)
		if restoreRegistersErr != syscall.Errno(0) {
			return
		}
		if sr.Fpregset != nil {
			iov := sys.Iovec{Base: &sr.Fpregset[0], Len: uint32(len(sr.Fpregset))}
			_, _, restoreRegistersErr = syscall.Syscall6(syscall.SYS_PTRACE, sys.PTRACE_SETREGSET, uintptr(t.ID), uintptr(elf.NT_FPREGSET), uintptr(unsafe.Pointer(&iov)), 0, 0)
		}
	})
	if restoreRegistersErr == syscall.Errno(0) {
		restoreRegistersErr = nil
	}
	return restoreRegistersErr
}

func pokeMemory(pid int, addr uintptr, data []byte) (count int, err error) {
	count = 0;
	f, err := os.OpenFile(fmt.Sprintf("/proc/%d/task/%d/mem", pid, pid), os.O_RDWR, 0)
	if err != nil {
		return 0, err
	}
	defer f.Close()
	_, err = f.Seek(int64(addr), os.SEEK_SET)
	if err != nil {
		return 0, err
	}
	count, err = f.Write(data)
	if count != len(data) || err != nil {
		return 0, err
	}
	return count, err
}

func flushCache(start uintptr, end uintptr) {
//	fmt.Printf("flushCache %x\n", start)
	_, _, err := syscall.Syscall(0x0f0002, start, end, 0)
	if err != syscall.Errno(0) {
		fmt.Printf("flushCache err=%x\n", err)
	}
}

// resolvePCForArm is used to resolve all next PC for current instruction.
func (t *nativeThread) resolvePC(savedRegs proc.Registers) ([]uint64, error) {
	regs := savedRegs.(*linutil.ARMRegisters)

	// Use ptrace to get better performance.
	nextInstrLen := t.BinInfo().Arch.MaxInstructionLength()
	nextInstrBytes := make([]byte, nextInstrLen)
	var err error
	t.dbp.execPtraceFunc(func() {
		_, err = sys.PtracePeekData(t.ID, uintptr(regs.PC()), nextInstrBytes)
	})
	if err != nil {
		return nil, err
	}
	nextPcs := []uint64{
		regs.PC() + uint64(nextInstrLen),
	}

	// There is some thing special on ARM platform, we use UND as break instruction.
	if bytes.Equal(nextInstrBytes, t.BinInfo().Arch.BreakpointInstruction()) {
		//fmt.Println("found armBreakInstruction")
		//nextPcs = append(nextPcs, regs.PC()+uint64(4))
		return nextPcs, nil
	}
	// Golang always use ARM mode.
	nextInstr, err := armasm.Decode(nextInstrBytes, armasm.ModeARM)
	if err != nil {
		return nil, err
	}
	switch nextInstr.Op {
	case armasm.BL, armasm.BLX, armasm.B, armasm.BX:
		switch arg := nextInstr.Args[0].(type) {
		case armasm.Imm:
			//fmt.Println("armasm.Imm")
			nextPcs = append(nextPcs, uint64(arg))
		case armasm.Reg:
			pc, err := regs.Get(int(arg))
			if err != nil {
				return nil, err
			}
			nextPcs = append(nextPcs, pc)
		case armasm.PCRel:
			nextPcs = append(nextPcs, regs.PC()+uint64(arg))
		}
	case armasm.POP:
		if regList, ok := nextInstr.Args[0].(armasm.RegList); ok && (regList&(1<<uint(armasm.PC)) != 0) {
			pc, err := regs.Get(int(armasm.SP))
			if err != nil {
				return nil, err
			}
			for i := 0; i < int(armasm.PC); i++ {
				if regList&(1<<uint(i)) != 0 {
					pc += uint64(nextInstrLen)
				}
			}
			pcMem := make([]byte, nextInstrLen)
			t.dbp.execPtraceFunc(func() {
				_, err = sys.PtracePeekData(t.ID, uintptr(pc), pcMem)
			})
			if err != nil {
				return nil, err
			}
			nextPcs = append(nextPcs, uint64(binary.LittleEndian.Uint32(pcMem)))
		}
	case armasm.LDR:
		// We need to check for the first args to be PC.
		if reg, ok := nextInstr.Args[0].(armasm.Reg); ok && reg == armasm.PC {
			switch arg := nextInstr.Args[1].(type) {
			case armasm.Mem:
				pc, err := regs.Get(int(arg.Base))
				if err != nil {
					return nil, err
				}
				if arg.Mode == armasm.AddrOffset || arg.Mode == armasm.AddrPreIndex {
					if arg.Sign != 0 {
						idx, err := regs.Get(int(arg.Index))
						if err != nil {
							return nil, err
						}
						if arg.Shift != armasm.ShiftLeft || arg.Count != 0 {
							switch arg.Shift {
							case armasm.ShiftLeft:
								idx <<= arg.Count
							case armasm.ShiftRight, armasm.ShiftRightSigned:
								idx >>= arg.Count
							case armasm.RotateRight, armasm.RotateRightExt:
								idx = bits.RotateLeft64(idx, int(-arg.Count))
							}
						}
						if arg.Sign < 0 {
							pc -= idx
						} else {
							pc += idx
						}
					} else {
						pc = uint64(int64(pc) + int64(arg.Offset))
					}
				}
				pcMem := make([]byte, nextInstrLen)
				t.dbp.execPtraceFunc(func() {
					_, err = sys.PtracePeekData(t.ID, uintptr(pc), pcMem)
				})
				if err != nil {
					return nil, err
				}
				nextPcs = append(nextPcs, uint64(binary.LittleEndian.Uint32(pcMem)))
			}
		}
	case armasm.MOV, armasm.ADD:
		// We need to check for the first args to be PC.
		if reg, ok := nextInstr.Args[0].(armasm.Reg); ok && reg == armasm.PC {
			var pc uint64
			for _, argRaw := range nextInstr.Args[1:] {
				switch arg := argRaw.(type) {
				case armasm.Imm:
					pc += uint64(arg)
				case armasm.Reg:
					regVal, err := regs.Get(int(arg))
					if err != nil {
						return nil, err
					}
					pc += regVal
				}
			}
			nextPcs = append(nextPcs, pc)
		}
	}
	return nextPcs, nil
}

func (t *nativeThread) singleStep() (err error) {
	// Arm don't have ptrace singleStep implemented, so we use breakpoint to emulate it.
	regs, err := t.Registers()
	if err != nil {
		return err
	}
	nextPcs, err := t.resolvePC(regs)
	if err != nil {
		return err
	}
	originalDatas := make(map[uintptr][]byte)
	// Do in batch, first set breakpoint, then continue.
	t.dbp.execPtraceFunc(func() {
		breakpointInstr := t.BinInfo().Arch.BreakpointInstruction()
		readWriteMem := func(i int, addr uintptr, instr []byte) error {
			originalData := make([]byte, len(breakpointInstr))
			_, err = sys.PtracePeekData(t.ID, addr, originalData)
			if err != nil {
				return err
			}
			_, err = pokeMemory(t.ID, addr, instr)
			//?? TODO_, err = sys.PtracePokeData(t.ID, addr, instr)
			if err != nil {
				return err
			}
			// Everything is ok, store originalData
			originalDatas[addr] = originalData
			return nil
		}
		for i, nextPc := range nextPcs {
			err = readWriteMem(i, uintptr(nextPc), breakpointInstr)
			if err != nil {
				return
			}
		}
	})
	// Make sure we restore before return.
	defer func() {
		// Update err.
		t.dbp.execPtraceFunc(func() {
			for addr, originalData := range originalDatas {
				if originalData != nil {
					//?? TODO_, err = sys.PtracePokeData(t.ID, addr, originalData)
					_, err = pokeMemory(t.ID, addr, originalData)
				}
			}
		})
	}()
	if err != nil {
		return err
	}
	for {
		sig := 0
		t.dbp.execPtraceFunc(func() {
			err = ptraceCont(t.ID, sig)
		})
		if err != nil {
			return err
		}		
	// To be able to catch process exit, we can only use wait instead of waitFast.
		wpid, status, err := t.dbp.wait(t.ID, 0)
		if err != nil {
			return err
		}
		if (status == nil || status.Exited()) && wpid == t.dbp.pid {
			t.dbp.postExit()
			rs := 0
			if status != nil {
				rs = status.ExitStatus()
			}
			return proc.ErrProcessExited{Pid: t.dbp.pid, Status: rs}
		}
		if wpid == t.ID {
			sig = 0
			switch s := status.StopSignal(); s {
			case sys.SIGTRAP:				
			/*
   				regs, err := t.Registers()
				if err != nil {
					return err
				}
				fmt.Printf("rcvd SIGTRAP at %x\n", regs.PC())
			*/	
				return nil
			case sys.SIGSTOP:
				// delayed SIGSTOP, ignore it
				fmt.Println("rcvd SIGSTOP")
			case sys.SIGILL, sys.SIGBUS, sys.SIGFPE, sys.SIGSEGV, sys.SIGSTKFLT:
				//fmt.Println("rcvd SIGILL...")
				// propagate signals that can have been caused by the current instruction
				fmt.Printf("rcvd other %s\n", s)
				sig = int(s)
			default:
				// delay propagation of all other signals
				fmt.Println("rcvd Other")
				t.os.delayedSignal = int(s)
			}
		}
	}
}

func (t *nativeThread) findHardwareBreakpoint() (*proc.Breakpoint, error) {
/*	var siginfo ptraceSiginfoArm
	var err error
	t.dbp.execPtraceFunc(func() {
		_, _, err = syscall.Syscall6(syscall.SYS_PTRACE, sys.PTRACE_GETSIGINFO, uintptr(t.ID), 0, uintptr(unsafe.Pointer(&siginfo)), 0, 0)
	})
	if err != syscall.Errno(0) {
		return nil, err
	}
	if siginfo.signo != uint32(sys.SIGTRAP) || (siginfo.code&0xffff) != _TRAP_HWBKPT {
		return nil, nil
	}

	for _, bp := range t.dbp.Breakpoints().M {
		if bp.WatchType != 0 && siginfo.addr >= bp.Addr && siginfo.addr < bp.Addr+uint64(bp.WatchType.Size()) {
			return bp, nil
		}
	}

	return nil, fmt.Errorf("could not find hardware breakpoint for address %#x", siginfo.addr)
*/
	return nil, fmt.Errorf("hw breakpoint not supported")
}

func (t *nativeThread) writeHardwareBreakpoint(addr uint64, wtype proc.WatchType, idx uint8) error {
/* 	wpstate, err := t.getWatchpoints()
	if err != nil {
		return err
	}
	if idx >= wpstate.num {
		return errors.New("hardware breakpoints exhausted")
	}

	const (
		readBreakpoint  = 0x1
		writeBreakpoint = 0x2
		lenBitOffset    = 5
		typeBitOffset   = 3
		privBitOffset   = 1
	)

	var typ uint64
	if wtype.Read() {
		typ |= readBreakpoint
	}
	if wtype.Write() {
		typ |= writeBreakpoint
	}

	len := uint64((1 << wtype.Size()) - 1) // arm wants the length expressed as address bitmask

	priv := uint64(3)

	ctrl := (len << lenBitOffset) | (typ << typeBitOffset) | (priv << privBitOffset) | 1
	wpstate.set(idx, addr, ctrl)

	return t.setWatchpoints(wpstate)
 */
	return fmt.Errorf("hw breakpoint not supported")
}

func (t *nativeThread) clearHardwareBreakpoint(addr uint64, wtype proc.WatchType, idx uint8) error {
/*	wpstate, err := t.getWatchpoints()
	if err != nil {
		return err
	}
	if idx >= wpstate.num {
		return errors.New("hardware breakpoints exhausted")
	}
	wpstate.set(idx, 0, 0)
	return t.setWatchpoints(wpstate)
*/
	return fmt.Errorf("hw breakpoint not supported")
}
