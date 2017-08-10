Summary:        Minimal C library to access, configure and get/send AER data from sensors or to/from neuromorphic processors.
Name:           libcaer
Version:        2.3.0
Release:        0
License:        BSD
Group:          unknown
Vendor:         iniLabs

Requires: libserialport >= 0.1.1, opencv >= 3.1.0, libusbx >= 1.0.17
Autoreq: 0

Prefix: /usr

%build
%cmake .
make %{?_smp_mflags}

%install
make install DESTDIR=%{buildroot}


