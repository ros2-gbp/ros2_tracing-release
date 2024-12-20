%bcond_without tests
%bcond_without weak_deps

%global __os_install_post %(echo '%{__os_install_post}' | sed -e 's!/usr/lib[^[:space:]]*/brp-python-bytecompile[[:space:]].*$!!g')
%global __provides_exclude_from ^/opt/ros/rolling/.*$
%global __requires_exclude_from ^/opt/ros/rolling/.*$

Name:           ros-rolling-tracetools-trace
Version:        8.5.0
Release:        1%{?dist}%{?release_suffix}
Summary:        ROS tracetools_trace package

License:        Apache 2.0
URL:            https://docs.ros.org/en/rolling/p/tracetools_trace/
Source0:        %{name}-%{version}.tar.gz

Requires:       ros-rolling-lttngpy
Requires:       ros-rolling-ros-workspace
BuildRequires:  python%{python3_pkgversion}-devel
BuildRequires:  ros-rolling-ros-workspace
Provides:       %{name}-devel = %{version}-%{release}
Provides:       %{name}-doc = %{version}-%{release}
Provides:       %{name}-runtime = %{version}-%{release}

%if 0%{?with_tests}
BuildRequires:  python%{python3_pkgversion}-pytest
BuildRequires:  ros-rolling-ament-copyright
BuildRequires:  ros-rolling-ament-flake8
BuildRequires:  ros-rolling-ament-mypy
BuildRequires:  ros-rolling-ament-pep257
BuildRequires:  ros-rolling-ament-xmllint
%endif

%description
Tools for setting up tracing sessions.

%prep
%autosetup -p1

%build
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/rolling/setup.sh" ]; then . "/opt/ros/rolling/setup.sh"; fi
%py3_build

%install
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/rolling/setup.sh" ]; then . "/opt/ros/rolling/setup.sh"; fi
%py3_install -- --prefix "/opt/ros/rolling"

%if 0%{?with_tests}
%check
# Look for a directory with a name indicating that it contains tests
TEST_TARGET=$(ls -d * | grep -m1 "\(test\|tests\)" ||:)
if [ -n "$TEST_TARGET" ] && %__python3 -m pytest --version; then
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/rolling/setup.sh" ]; then . "/opt/ros/rolling/setup.sh"; fi
%__python3 -m pytest $TEST_TARGET || echo "RPM TESTS FAILED"
else echo "RPM TESTS SKIPPED"; fi
%endif

%files
/opt/ros/rolling

%changelog
* Fri Dec 20 2024 Christophe Bedard <bedard.christophe@gmail.com> - 8.5.0-1
- Autogenerated by Bloom

* Mon Nov 25 2024 Christophe Bedard <bedard.christophe@gmail.com> - 8.4.1-1
- Autogenerated by Bloom

* Tue Oct 15 2024 Christophe Bedard <bedard.christophe@gmail.com> - 8.4.0-1
- Autogenerated by Bloom

* Fri Apr 26 2024 Christophe Bedard <bedard.christophe@gmail.com> - 8.3.0-1
- Autogenerated by Bloom

* Tue Apr 16 2024 Christophe Bedard <bedard.christophe@gmail.com> - 8.2.0-1
- Autogenerated by Bloom

* Thu Mar 28 2024 Christophe Bedard <bedard.christophe@gmail.com> - 8.1.0-1
- Autogenerated by Bloom

* Wed Mar 06 2024 Christophe Bedard <bedard.christophe@gmail.com> - 8.0.0-2
- Autogenerated by Bloom

