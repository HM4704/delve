package regnum

import (
	"fmt"
)

// The mapping between hardware registers and DWARF registers is specified
// in the DWARF for the ARM® Architecture page 7,
// Table 1
// http://infocenter.arm.com/help/topic/com.arm.doc.ihi0040b/IHI0040B_aadwarf.pdf

const (
	ARM_X0         = 0  // X1 through X30 follow
	ARM_BP         = 11 // also X29
	ARM_LR         = 30 // also X30
	ARM_SP         = 13
	ARM_PC         = 15
	ARM_V0         = 64 // V1 through V31 follow
	_ARM_MaxRegNum = ARM_V0 + 31
)

func ARMToName(num uint64) string {
	switch {
	case num <= 30:
		return fmt.Sprintf("X%d", num)
	case num == ARM_SP:
		return "SP"
	case num == ARM_PC:
		return "PC"
	case num >= ARM_V0 && num <= 95:
		return fmt.Sprintf("V%d", num-64)
	default:
		return fmt.Sprintf("unknown%d", num)
	}
}

func ARMMaxRegNum() uint64 {
	return _ARM_MaxRegNum
}

var ARMNameToDwarf = func() map[string]int {
	r := make(map[string]int)
	for i := 0; i <= 32; i++ {
		r[fmt.Sprintf("x%d", i)] = ARM_X0 + i
	}
	r["fp"] = 29
	r["lr"] = 30
	r["sp"] = 31
	r["pc"] = 32

	for i := 0; i <= 31; i++ {
		r[fmt.Sprintf("v%d", i)] = ARM_V0 + i
	}

	return r
}()
