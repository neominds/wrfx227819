# This file will be included by default template of
# the wr-kernel layer conditionally, depend the
# policy/implementation there.
#
# The packages specified in this file will NOT necessarily
# present in the target rootfs, read the default template
# of wr-kernel layer:
#
#   wr-kernel/templates/default/template.conf
#
# to check if target rootfs type is blacklisted or not.

IMAGE_INSTALL += "alsa-utils"
IMAGE_INSTALL += "alsa-lib"
IMAGE_INSTALL += "i2c-tools"
