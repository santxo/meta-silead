require conf/distro/poky.conf
require conf/distro/include/arm-defaults.inc

DISTRO = "tdx-base"
DISTRO_NAME = "Toradex Distro Base"

TDX_BUILDNBR ?= "0"
TDX_PURPOSE ?= "Testing"

def get_tdx_prerelease(purpose, datetime):
    if purpose == "Testing":
        return '-devel-%s' % (datetime)
    elif purpose == "Nightly":
        return '-devel-%s' % (datetime[:8])
    elif purpose == "Monthly":
        return '-devel-%s' % (datetime[:6])
    elif purpose == "Release":
        return ''
    else:
        return '-devel-%s' % (datetime)

TDX_BUILD ?= "+build.${TDX_BUILDNBR}"
TDX_MATRIX_BUILD_TIME ?= "${DATETIME}"
TDX_MATRIX_BUILD_TIME[vardepsexclude] = "DATETIME"
TDX_PRERELEASE ?= "${@get_tdx_prerelease(d.getVar('TDX_PURPOSE'), d.getVar('TDX_MATRIX_BUILD_TIME'))}"
TDX_RELEASE ?= "0.0.0"

# Toradex Version number without date for U-Boot/Kernel and other Toradex
# specific software artifacts.
TDX_VERSION ?= "${TDX_RELEASE}${@'' if d.getVar('TDX_PURPOSE') == 'Release' else '-devel'}"

# Complete image version with date and build number
DISTRO_VERSION = "${TDX_RELEASE}${TDX_PRERELEASE}${TDX_BUILD}"

SDK_VENDOR = "-tdxsdk"
SDK_NAME_PREFIX = "${DISTRO}"

MAINTAINER = "Toradex <yocto-patches@toradex.com>"

TARGET_VENDOR = "-tdx"

DISTROOVERRIDES = "tdx"

# create etc/build in the rootfs
INHERIT += "image-buildinfo"

# Log information on images and packages
INHERIT += "buildhistory"
BUILDHISTORY_COMMIT = "1"

# Use bluez5 as default.
DISTRO_FEATURES_append = " bluez5"

IMAGE_LINGUAS ?= "en-us"

# TODO: review default distro features
DISTRO_FEATURES_append = " pam systemd"
DISTRO_FEATURES_remove = "sysvinit"
VIRTUAL-RUNTIME_init_manager = "systemd"
PACKAGECONFIG_append_pn-qemu-native = " libusb"

# Use i.MX Gstreamer Version (for PXP DMA-ENGINE Driver)
PREFERRED_VERSION_gstreamer1.0_imx = "1.16.imx"
PREFERRED_VERSION_gstreamer1.0-plugins-base_imx = "1.16.imx"
PREFERRED_VERSION_gstreamer1.0-plugins-good_imx = "1.16.imx"
PREFERRED_VERSION_gstreamer1.0-plugins-bad_imx = "1.16.imx"

# these 1.14.imx variants have their COMPATIBLE set so that they don't build
# for use-mainline-bsp
PREFERRED_VERSION_gstreamer1.0_use-mainline-bsp = "1.16%"
PREFERRED_VERSION_gstreamer1.0-plugins-base_use-mainline-bsp = "1.16%"
PREFERRED_VERSION_gstreamer1.0-plugins-good_use-mainline-bsp = "1.16%"
PREFERRED_VERSION_gstreamer1.0-plugins-bad_use-mainline-bsp = "1.16%"
