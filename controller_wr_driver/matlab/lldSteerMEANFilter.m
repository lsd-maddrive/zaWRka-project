window = 31; 
MEAN = []; 
m_sum = 0;
raw_size = length(RAW); 

% to match sizes of arrays
for f = 1:1:(int16(window/2) - 1)
    m = RAW(f, 1);
    MEAN = [MEAN; m]; 
end

for i = int16(window/2):1:(raw_size - int16(window/2) - 1)
    for j = 1:1:(int16(window/2)-1)
        m_sum = m_sum + RAW(i - (j),1);
        m_sum = m_sum + RAW(i + (j),1);
    end
    m_sum = m_sum + RAW(i ,1);
    m = m_sum / window; 
    m_sum = 0;

    MEAN = [MEAN; m]; 
end

% to match sizes of arrays
for g = (raw_size - int16(window/2)):1:raw_size
    m = RAW(g, 1);
    MEAN = [MEAN; m]; 
end

hold on
plot(MEAN)
grid on 