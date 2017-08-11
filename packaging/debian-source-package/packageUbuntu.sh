#!/bin/sh

# Requirements: Ubuntu, GPG key (seahorse), git, devscripts, build-essential

GPG_PUBLIC_KEY=0x058B659E
PKG_NAME=libcaer
PKG_VERSION=2.3.0
PKG_RELEASE=1
DISTRO=xenial
BRANCH=master
PPA_REPO=llongi/inilabs
DATE=$(LC_ALL=C date +'%a, %d %b %Y %T %z')
CUR_DIR=$(pwd)
BASE_DIR="$CUR_DIR/../../"
BUILD_DIR="$CUR_DIR/build/"
PKG_BUILD_DIR="$BUILD_DIR/$PKG_NAME-$PKG_VERSION/"
DEBIAN_DIR="$PKG_BUILD_DIR/debian/"
UPLOAD="false"
DEBUILD_ARGS="-k$GPG_PUBLIC_KEY -sa"

while test $# -gt 0
do
    case "$1" in
        --distro) DISTRO="$2"
            ;;
        --source) DEBUILD_ARGS="-S $DEBUILD_ARGS"
            ;;
        --upload) UPLOAD="true"
            ;;
        --no-orig-tar-upload) DEBUILD_ARGS="$DEBUILD_ARGS -sd"
            ;;
    esac
    shift
done

echo "Started the debian source packaging process for distro $DISTRO"

rm -rf "$BUILD_DIR"
mkdir -p "$BUILD_DIR"

# Create the tar.gz containing the source and extract it
cd "$BASE_DIR"
git archive --format tar.gz --output "$BUILD_DIR/${PKG_NAME}_${PKG_VERSION}.orig.tar.gz" --prefix="${PKG_NAME}-${PKG_VERSION}/" "$BRANCH"

cd "$BUILD_DIR"
tar -xzf "${PKG_NAME}_${PKG_VERSION}.orig.tar.gz"

mkdir -p "$DEBIAN_DIR"

# Copy correct control and rules files for distro
cp "$CUR_DIR/$DISTRO/control" "$DEBIAN_DIR/control"
cp "$CUR_DIR/$DISTRO/rules"   "$DEBIAN_DIR/rules"

# Copy copyright file (use main license)
cp "$BASE_DIR/COPYING" "$DEBIAN_DIR/copyright"

# Formats file
mkdir -p "$DEBIAN_DIR/source/"
echo "3.0 (quilt)" > "$DEBIAN_DIR/source/format"

# Create the changelog file for the distro
CHANGELOG_FILE="$DEBIAN_DIR/changelog"
echo "$PKG_NAME ($PKG_VERSION-$PKG_RELEASE~$DISTRO) $DISTRO; urgency=low" > "$CHANGELOG_FILE"
echo "" >> "$CHANGELOG_FILE"
echo "  * Released $PKG_NAME version $PKG_VERSION for distro $DISTRO." >> "$CHANGELOG_FILE"
echo "" >> "$CHANGELOG_FILE"
echo " -- iniLabs <support@inilabs.com>  $DATE" >> "$CHANGELOG_FILE"

# Launch debuild
cd "$PKG_BUILD_DIR"
debuild $DEBUILD_ARGS

# Send to Launchpad PPA
if [ "$UPLOAD" = "true" ]; then
	cd "$BUILD_DIR"
	dput ppa:$PPA_REPO "${PKG_NAME}_${PKG_VERSION}-${PKG_RELEASE}~${DISTRO}_source.changes"
fi;
