# usage: source autolink_tools_auto_complete.bash

function _autolink_launch_complete() {
  COMPREPLY=()
  local word=${COMP_WORDS[COMP_CWORD]}
  local cmd=${COMP_WORDS[COMP_CWORD-1]}
  case $cmd in
  'autolink_launch')
    COMPREPLY=( $(compgen -W "start stop" -- ${word}) )
    ;;
  'start')
    compopt -o nospace
    local files=`ls *.launch 2>/dev/null`
    COMPREPLY=( $(compgen -W "$files" -- ${word}) )
    ;;
  'stop')
    compopt -o nospace
    local files=`ls *.launch 2>/dev/null`
    COMPREPLY=( $(compgen -W "$files" -- ${word}) )
    ;;
  *)
    ;;
  esac
}

function _autolink_recorder_complete() {
  COMPREPLY=()
  local word=${COMP_WORDS[COMP_CWORD]}
  local cmd=${COMP_WORDS[COMP_CWORD-1]}
  case $cmd in
  'autolink_recorder')
    COMPREPLY=( $(compgen -W "play info record split recover" -- ${word}) )
    ;;
  *)
    ;;
  esac
}

function _autolink_channel_complete() {
  COMPREPLY=()
  local word=${COMP_WORDS[COMP_CWORD]}
  local cmd=${COMP_WORDS[COMP_CWORD-1]}
  case $cmd in
  'autolink_channel')
    COMPREPLY=( $(compgen -W "echo list info hz bw type" -- ${word}) )
    ;;
  *)
    ;;
  esac
}

function _autolink_node_complete() {
  COMPREPLY=()
  local word=${COMP_WORDS[COMP_CWORD]}
  local cmd=${COMP_WORDS[COMP_CWORD-1]}
  case $cmd in
  'autolink_node')
    COMPREPLY=( $(compgen -W "list info" -- ${word}) )
    ;;
  *)
    ;;
  esac
}

function _autolink_service_complete() {
  COMPREPLY=()
  local word=${COMP_WORDS[COMP_CWORD]}
  local cmd=${COMP_WORDS[COMP_CWORD-1]}
  case $cmd in
  'autolink_service')
    COMPREPLY=( $(compgen -W "list info" -- ${word}) )
    ;;
  *)
    ;;
  esac
}
complete -F _autolink_launch_complete -o default autolink_launch
complete -F _autolink_recorder_complete -o default autolink_recorder
complete -F _autolink_channel_complete -o default autolink_channel
complete -F _autolink_node_complete -o default autolink_node
complete -F _autolink_service_complete -o default autolink_service
