#!/bin/sh

GPG_PUBLIC_KEY=0x058B659E
PKG_NAME=libcaer
PKG_VERSION=2.3.0
DISTRO=xenial
BRANCH=master
DATE=$(LC_ALL=C date +'%a, %d %b %Y %T %z')
CUR_DIR=$(pwd)
BASE_DIR="$CUR_DIR/../../"
BUILD_DIR="$CUR_DIR/$PKG_NAME/"
DEBIAN_DIR="$BUILD_DIR/debian/"
UPLOAD="true"
DEBUILD_ARGS=""

while test $# -gt 0
do
    case "$1" in
        --distro) DISTRO="$2"
            ;;
        --no-upload) UPLOAD="false"
            ;;
        --no-orig-tar-upload) DEBUILD_ARGS="$DEBUILD_ARGS -sd"
            ;;
    esac
    shift
done

echo "Started the debian source packaging process for distro $DISTRO"

rm -rf "$DEBIAN_DIR"
mkdir -p "$DEBIAN_DIR"

# Copy correct control and rules files for distro
cp "$CUR_DIR/$DISTRO/control" "$DEBIAN_DIR/control"
cp "$CUR_DIR/$DISTRO/rules"   "$DEBIAN_DIR/rules"

# Copy copyright file (use main license)
cp "$BASE_DIR/COPYING" "$DEBIAN_DIR/copyright"

# Create the changelog file for the distro
CHANGELOG_FILE="$DEBIAN_DIR/changelog"
echo "$PKG_NAME ($PKG_VERSION-0~$DISTRO) $DISTRO; urgency=low" > "$CHANGELOG_FILE"
echo "" >> "$CHANGELOG_FILE"
echo "  * Released $PKG_NAME version $PKG_VERSION." >> "$CHANGELOG_FILE"
echo "" >> "$CHANGELOG_FILE"
echo " -- iniLabs <support@inilabs.com>  $DATE" >> "$CHANGELOG_FILE"

# Create the tar.gz containing the source
cd "$BASE_DIR"
git archive --format tar.gz --output "$BUILD_DIR/${PKG_NAME}_${PKG_VERSION}.orig.tar.gz" --prefix=$PKG_NAME-$PKG_VERSION/ $BRANCH

exit

# Launch debuild
cd "$BUILD_DIR"
debuild -S -sa -k$GPG_PUBLIC_KEY $DEBUILD_ARGS

# Send to Launchpad PPA
if [[ "$UPLOAD" = "true" ]]; then
	cd ..
	dput ppa:llongi/inilabs "${PKG_NAME}_${PKG_VERSION}-0~${DISTRO}"_source.changes
fi;
