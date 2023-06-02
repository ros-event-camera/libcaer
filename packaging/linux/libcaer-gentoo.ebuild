# Copyright 2021 iniVation AG
# Distributed under the terms of the GNU General Public License v2

EAPI=7

inherit eutils cmake udev

DESCRIPTION="Minimal C library to access, configure and get data from neuromorphic sensors and processors."
HOMEPAGE="https://gitlab.com/inivation/dv/${PN}/"

SRC_URI="https://release.inivation.com/${PN}/${P}.tar.gz"

LICENSE="BSD-2"
SLOT="0"
KEYWORDS="amd64 x86 arm64 arm"
IUSE="debug +serialdev +opencv static-libs"

RDEPEND=">=dev-libs/libusb-1.0.17
	serialdev? ( >=dev-libs/libserialport-0.1.1 )
	opencv? ( >=media-libs/opencv-3.1.0 )"

DEPEND="${RDEPEND}
	virtual/pkgconfig
	>=dev-util/cmake-3.10.0"

src_configure() {
	local mycmakeargs=(
		-DENABLE_SERIALDEV="$(usex serialdev 1 0)"
		-DENABLE_OPENCV="$(usex opencv 1 0)"
		-DENABLE_STATIC="$(usex static-libs 1 0)"
		-DUDEV_INSTALL=1
	)

	cmake_src_configure
}

pkg_postinst() {
	udev_reload
}
