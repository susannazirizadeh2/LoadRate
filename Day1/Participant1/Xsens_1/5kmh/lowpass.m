function yFilt=lowpass(y,RC,dt)

yFilt=y;
alphaLow=dt/(RC+dt);
for i=2:length(y)
    yFilt(i)=alphaLow.*y(i) +(1-alphaLow).*yFilt(i-1);
end