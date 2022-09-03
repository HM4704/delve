// This file is used to detect build on unsupported GOOS/GOARCH combinations.

<<<<<<< HEAD
//+build !linux,!darwin,!windows,!freebsd linux,!amd64,!arm64,!386,!arm darwin,!amd64 windows,!amd64 freebsd,!amd64
=======
//go:build (!linux && !darwin && !windows && !freebsd) || (linux && !amd64 && !arm64 && !386) || (darwin && !amd64 && !arm64) || (windows && !amd64) || (freebsd && !amd64)
// +build !linux,!darwin,!windows,!freebsd linux,!amd64,!arm64,!386 darwin,!amd64,!arm64 windows,!amd64 freebsd,!amd64
>>>>>>> b19d67ccf239ba2bb0ca55cae20775cf8c51c771

package your_operating_system_and_architecture_combination_is_not_supported_by_delve
