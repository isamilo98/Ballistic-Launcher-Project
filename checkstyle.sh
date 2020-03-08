#!/bin/bash

for f in src/ca/mcgill/ecse211/project/*.java; do
    java -jar lib/checkstyle-8.24-all.jar -c config/checkstyle/checkstyle.xml ${f}
done
