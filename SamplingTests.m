load('Fits/FourierAff.mat');
load('Fits/PolyAff.mat');
load('Fits/SplineAff.mat');

numTests = 10000;
bounds = [10^-3.5, 10^1.2];
CTLR = Controller(0,300);
Orig = @(step) 66.04;
Basic = @(step) 53;
Poly = @(step) feval(PolyFit, log10(step));
Fourier = @(step) feval(FourierFit, log10(step));
Spline = @(step) feval(SplineModel, log10(step));

Aorig = Peval(CTLR, Orig, 'A', numTests, bounds)
ABasic = Peval(CTLR, Basic, 'A', numTests, bounds)
Alow = Peval(CTLR,Poly,'A',numTests,bounds)
Amed = Peval(CTLR,Fourier,'A',numTests,bounds)
Ahigh = Peval(CTLR,Spline,'A',numTests,bounds)

results = table(numTests, Aorig, ABasic, Alow, Amed, Ahigh)
save('Results/GridSearchRobustPerformance')