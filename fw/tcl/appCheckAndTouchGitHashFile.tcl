proc getGitHash {} {
  if { [exec sh -c {git update-index && git diff-index --quiet HEAD; echo $?}] != 0 } {
    set git_hash {00000000}
    puts "GIT NOT CLEAN - not providing a hash"
  } else {
    set git_hash "[exec git rev-parse --short=8 HEAD]"
  }
  return "${git_hash}"
}

proc appCheckAndTouchGitHashFile { fn board_version } {

  set git_hash [getGitHash]

  if { ! [file exists "${fn}"] || ( [exec sh -c "grep -q '${git_hash}' '${fn}'; echo \$?"] != 0 ) } {
    set fp [open "${fn}" w+]
    puts $fp "-- AUTOMATICALLY GENERATED; DO NOT EDIT"
    puts $fp "library ieee;"
    puts $fp "use     ieee.std_logic_1164.all;"
    puts $fp "package GitVersionPkg is"
    puts $fp "   constant GIT_VERSION_C   : std_logic_vector(31 downto 0) := x\"${git_hash}\";"
    puts $fp "   constant BOARD_VERSION_C : std_logic_vector(31 downto 0) := x\"${board_version}\";"
    puts $fp "end package GitVersionPkg;"
    close $fp
  }
  puts ${git_hash}

  return "${git_hash}"
}
