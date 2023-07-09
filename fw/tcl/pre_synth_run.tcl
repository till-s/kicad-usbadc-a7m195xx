puts "USER PRE_SYNTH_RUN script"

set origin_dir [file dirname [info script]]

# apparently utils_1 fileset is not set up from where we are called
#source [get_files -of_objects [get_filesets utils_1] {*/appCheckAndTouchGitHashFile.tcl}]
source "${origin_dir}/../tcl/appCheckAndTouchGitHashFile.tcl"

set git_hash [getGitHash]

set genericArgList [get_property generic [current_fileset]]
lappend genericArgList "GIT_HASH_G=32'h${git_hash}"

set_property generic ${genericArgList} [current_fileset]

# Must not use tcl_pre to generate version file; vivado
# checks dependencies before running this script and the
# synthesis goes out of date as soon as we touch the GitVersionPkg.vhd :-(
# appCheckAndTouchGitHashFile  "${origin_dir}/../hdl/GitVersionPkg.vhd" "01"
