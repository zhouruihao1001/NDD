function [ theta ] = XY2Theta( x, y )

    if (x >= 0 && y >= 0) 
        theta = 180/pi * std::atan(y/x);
    end
    if (x < 0 && y >= 0) 
        theta = 180 - ((180/pi) * std::atan(y/(-x)));
    end
    if (x < 0 && y < 0) 
        theta = 180 + ((180/pi) * std::atan(y/x));
    end
    if ( x >= 0 && y < 0)
        theta = 360 - ((180/pi) * std::atan((-y)/x));
    end

end

