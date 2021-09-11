function L = plucker(a,b)
l0 = a(1)*b(2) - b(1)*a(2);
l1 = a(1)*b(3) - b(1)*a(3);
l2 = a(1) - b(1);
l3 = a(2)*b(3) - b(2)*a(3);
l4 = a(3) - b(3);
l5 = b(2) - a(2);
L = [l0 l1 l2 l3 l4 l5];
end

