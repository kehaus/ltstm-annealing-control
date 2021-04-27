#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Apr 27 20:34:41 2021

@author: kh
"""


import u3

from de_power_supply import DEPowerSupply, ES03010



d = u3.U3()
es = DEPowerSupply(d)
