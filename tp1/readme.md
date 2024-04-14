Dado un controlador PI (para que tenga error nulo ante referencia de tipo escalón
1) Tenemos el padé para 48 segundos (límite)
2) Identificamos la frecuencia a la cual este padé aporta 5° (0.000794 Hz -> sumar 17.6 dB)
3) Aumentamos la ganancia para que el cruce por cero de L se de a la izquierda de esa frecuencia. Esto hace que aumente la rapidez con la que reacciona el sistema
4) Como aumentar la ganancia nos va a reducir el margen de fase (lo llevó a 20.2), agregamos una red de adelanto de fase [un cero y un polo a la derecha del cero]