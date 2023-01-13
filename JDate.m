function [jd] = JDate(y,m,d,h,min,s)
%Julian Date Converter

jd = 367 * y - fix(7 * (y + fix ((m + 9) / 12) ) /4) + fix(275 * m / 9)...
    + (h + min/60 + s/3600)/24 + d + 1721013.5; 
end