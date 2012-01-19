cmake_minimum_required(VERSION 2.8)
project(rxbag)

install(
  FILES scripts/rxbag
  DESTINATION share/rxbag/bin
  )

install(DIRECTORY icons
  DESTINATION share/rxbag/
  )

