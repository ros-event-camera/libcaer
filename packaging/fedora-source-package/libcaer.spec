Summary: Minimal C library to access, configure and get/send AER data from sensors or to/from neuromorphic processors.
Name:    libcaer
Version: 2.4.2
Release: 0%{?dist}
License: BSD
URL:     https://github.com/inilabs/libcaer/
Vendor:  iniLabs

BuildRequires: cmake >= 2.6.0, pkgconfig >= 0.29.0, libusbx-devel >= 1.0.17, libserialport-devel >= 0.1.1, opencv-devel >= 3.1.0
Requires: libusbx >= 1.0.17, libserialport >= 0.1.1, opencv >= 3.1.0

Source0: https://github.com/inilabs/libcaer/archive/%{version}.tar.gz

%description
Minimal C library to access, configure and get/send AER data from sensors or to/from neuromorphic processors.
Supported devices are the Dynamic Vision Sensor (DVS), all the DAVIS cameras, and the Dynap-se neuromorphic processor.

%prep
%autosetup

%build
%cmake -DENABLE_SERIALDEV=1 -DENABLE_OPENCV=1 .
make %{?_smp_mflags}

%install
make install DESTDIR=%{buildroot}

%files
%{_libdir}/*
%{_includedir}/*
