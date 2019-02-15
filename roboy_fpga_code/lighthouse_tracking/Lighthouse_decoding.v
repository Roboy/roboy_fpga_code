#include <stdio.h>

int main(int argc, char **argv, char **env) {
	if(argc<3) {
		printf("[file] [ch]\n");
		return 0;
	}

	FILE *f = fopen(argv[1],"rb");
	int ch = 0;
	sscanf(argv[2],"%d",&ch);

	unsigned char buf[1024];
	int bitmask = 1<<ch;
	double fil[7] = {0};
	double old = 0;
	int dist = 0;

	int taps[] = {	0xD3675,0x90C2D, // 8_seconds_8_bit_fixed_Data_20180805134454.8.dat _CHAN_INIT_Data_20180805155528.8.dat
						0xB85F3,0xC058B, // _CHAN_AFTER_BUTTON_Data_20180805170303.8.dat
					 };
	const int numtaps = sizeof(taps)/sizeof(taps[0]);
	int detect[numtaps] = {0};
	unsigned int sh = 0;
 
	while(1) {
		int n = fread(buf,1,sizeof(buf),f);
		int on = 0;
		
		if (n<1) {
			break;
		}
		
		for(int i=0; i<n; i++) {
			double in = (buf[i]&bitmask)?1.0:-1.0;
			// Filter it using simple FIR to get rid of some one-sample glitches around edges
			fil[0] = fil[1];
			fil[1] = fil[2];
			fil[2] = fil[3];
			fil[3] = fil[4];
			fil[4] = fil[5];
			fil[5] = in;
			double v = fil[0]*0.0280+fil[1]*0.1597+fil[2]*0.2779+fil[3]*0.3396+fil[4]*0.2779+fil[5]*0.1597+fil[6]*0.0280;
			double diff = v*old;
			old = v;
			dist++;
			
			if(diff<0 && dist>10) {		// 10 = little bit more than one symbol length
				if(dist>100) {				// long run of same bits between bursts
					int max = 0;
					int maxat = 0;
					for(int j=0; j<numtaps; j++) {
						//printf("%06X:%08d   ",taps[j],detect[j]); // detailed LFSR report
						if(detect[j]>max) {
							max = detect[j];
							maxat = j;
						}
					}
					printf("found LFSR %06X bit %d", taps[maxat], maxat&1);	// print best one to get bits
					printf("\n");
					for(int k=0; k<numtaps; k++) {
						detect[k] = 0;
					}
				} else {
					int bit = (v>0)?1:0;
					//printf("%d",bit); // print raw demodulated bits if you want that
					
					sh = (sh<<1)|bit;
					for(int j=0; j<numtaps; j++) {
						unsigned int b = sh&taps[j];
						b ^= b>>16;
						b ^= b>>8;
						b ^= b>>4;
						b ^= b>>2;
						b ^= b>>1;
						b &= 1;			// parity
						if(!b) {
							detect[j]++;	// count zeroes
						}
					}
				}
				dist=0;
			}
		}
	}
	fclose(f);
	return 0;
}