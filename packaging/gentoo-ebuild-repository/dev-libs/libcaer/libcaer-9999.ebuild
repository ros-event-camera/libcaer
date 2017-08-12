# Copyright 2017 iniLabs Ltd.
# Distributed under the terms of the GNU General Public License v2

EAPI=6

inherit eutils cmake-utils git-r3

DESCRIPTION="Fast, multi-threaded malloc() and nifty performance analysis tools"
HOMEPAGE="https://github.com/inilabs/${PN}/"

EGIT_REPO_URI="https://github.com/inilabs/${PN}.git"

LICENSE="BSD-2"
SLOT="0"
KEYWORDS="~amd64 ~arm ~x86"
IUSE="+serialdev opencv"

RDEPEND=">=dev-libs/libusb-1.0.17
	serialdev? ( >=dev-libs/libserialport-0.1.1 )
	opencv? ( >=media-libs/opencv-3.1.0 )"

DEPEND="${RDEPEND}
	virtual/pkgconfig
	>=dev-util/cmake-2.6"

src_configure() {
	local mycmakeargs=(
		-DENABLE_SERIALDEV="$(usex serialdev 1 0)"
		-DENABLE_OPENCV="$(usex opencv 1 0)"
	)

	cmake-utils_src_configure
}
