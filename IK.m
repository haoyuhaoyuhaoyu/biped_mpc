function pB = IK(pA, pC)
    l1 = 0.3;
    l2 = 0.3;
    [pB(1), pB(2)] = Get1From2(pC(1), pC(2), pA(1), pA(2), l1, l2, 'xSmaller');
end


