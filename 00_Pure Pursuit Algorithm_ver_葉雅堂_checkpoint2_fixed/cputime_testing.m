a= 0;
while(1)
    if a == 0
        t0 = cputime;
        a = a+1;
    end
    display(t0);
    display(cputime);

    a = 1;
    if cputime-t0 <=0
        break;
    end

end
display(cputime-t0 )