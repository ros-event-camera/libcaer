#!/bin/sh

PKG_NAME=
PKG_VERSION=0

while test $# -gt 0
do
    case "$1" in
        --pkg-name) PKG_NAME="$2"
            ;;
        --pkg-version) PKG_VERSION="$2"
            ;;
    esac
    shift
done

SHA256_FILE=$(sha256sum "/var/cache/distfiles/${PKG_NAME}-${PKG_VERSION}.tar.gz" | awk '{ print $1 }')

git clone "ssh://aur@aur.archlinux.org/${PKG_NAME}.git"

sed -e "s|VERSION_REPLACE|${PKG_VERSION}|g" -i ${PKG_NAME}-arch.pkgbuild
sed -e "s|SHA256SUM_REPLACE|${SHA256_FILE}|g" -i ${PKG_NAME}-arch.pkgbuild
cp ${PKG_NAME}-arch.pkgbuild ${PKG_NAME}/PKGBUILD

cd ${PKG_NAME}/
chmod 0777 .
sudo -u nobody makepkg --printsrcinfo > .SRCINFO
git add PKGBUILD .SRCINFO
git commit -m "${PKG_NAME}: new release ${PKG_VERSION}."
git push
