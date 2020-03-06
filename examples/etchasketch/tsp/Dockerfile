FROM python:2.7

ADD http://www.math.uwaterloo.ca/tsp/concorde/downloads/codes/linux24/linkern.gz /
RUN gunzip /linkern.gz
RUN chmod a+rwx /linkern
RUN mkdir -p /usr/local/bin
RUN mv /linkern /usr/local/bin

ADD https://raw.githubusercontent.com/evil-mad/EggBot/master/other/TSP-stipple/tsp_art_tools/tspart.py /
ADD https://raw.githubusercontent.com/evil-mad/EggBot/master/other/TSP-stipple/tsp_art_tools/tspbitcity.py /
ADD https://raw.githubusercontent.com/evil-mad/EggBot/master/other/TSP-stipple/tsp_art_tools/tspsolution.py /

ENTRYPOINT python tspart.py