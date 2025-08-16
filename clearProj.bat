del /Q MDK\Listings\*.*
del /Q MDK\Objects\*.*
del /Q MDK\*.bak
del /Q MDK\*.plg
del /Q MDK\*.dep
del /Q MDK\*.Administrator
del /Q MDK\*.zhang
del /Q MDK\*.scvd
del /Q output\list\*.map
del /Q output\exe\*

rd /s/q DebugConfig
rd /s/q Listings
rd /s/q Objects

exit