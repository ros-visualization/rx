cmake_minimum_required(VERSION 2.8)
project(rxbag)

install(PROGRAMS scripts/rxbag
        DESTINATION share/rxbag/bin)

install(DIRECTORY icons
  DESTINATION share/rxbag/
  )

