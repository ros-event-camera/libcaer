%global __cmake_in_source_build 1
%global debug_package %{nil}

Summary: Minimal C library to interact with neuromorphic sensors and processors
Name:    libcaer
Version: VERSION_REPLACE
Release: 0%{?dist}
License: BSD
URL:     https://gitlab.com/inivation/dv/libcaer/
Vendor:  iniVation AG

Source0: https://release.inivation.com/libcaer/%{name}-%{version}.tar.gz

BuildRequires: gcc >= 10.0, gcc-c++ >= 10.0, cmake >= 3.10, pkgconfig >= 0.29.0, libusbx-devel >= 1.0.17, libserialport-devel >= 0.1.1, opencv-devel >= 3.2.0
Requires: libusbx >= 1.0.17, libserialport >= 0.1.1, opencv >= 3.2.0

%description
Minimal C library to access, configure and get data from neuromorphic sensors
and processors. Currently supported devices are the Dynamic Vision Sensor
(DVS), the DAVIS cameras, and the Dynap-SE neuromorphic processor.

%package devel
Summary: Minimal C library to interact with neuromorphic sensors and processors (development files)
Requires: %{name}%{?_isa} = %{version}-%{release}, cmake >= 3.10, pkgconfig >= 0.29.0, opencv-devel >= 3.2.0

%description devel
Development files for libcaer, such as headers, pkg-config files, etc..

%prep
%autosetup

%build
%cmake -DENABLE_STATIC=1 -DENABLE_OPENCV=1 -DENABLE_SERIALDEV=1 -DUDEV_INSTALL=1
%cmake_build

%install
export QA_RPATHS=$(( 0x0001|0x0010 ))
%cmake_install

%files
%{_udevrulesdir}/*.rules
%{_libdir}/libcaer.so.*

%files devel
%{_includedir}/libcaer/
%{_includedir}/libcaercpp/
%{_datarootdir}/caer/
%{_libdir}/libcaer.so
%{_libdir}/libcaer.a
%{_libdir}/pkgconfig/
%{_libdir}/cmake/libcaer/

%changelog
* Mon Apr 20 2020 iniVation AG <support@inivation.com> - VERSION_REPLACE
- See ChangeLog file in source or GitLab.
