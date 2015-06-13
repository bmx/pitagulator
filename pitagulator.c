#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <getopt.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <errno.h>

#ifdef MIN
# undef MIN
#endif
#ifdef MAX
# undef MAX
#endif
#define MIN(a,b) ((a)<(b) ? (a) : (b))
#define MAX(a,b) ((a)>(b) ? (a) : (b))

#define MAX_CHAN 24

unsigned int PERI_BASE_ADDR = 0;

#define BCM2708_PERI_BASE       PERI_BASE_ADDR
#define GPIO_BASE              (BCM2708_PERI_BASE + 0x200000)

int  mem_fd;
void *gpio_map;

/* I/O access */
volatile unsigned *gpio;
/* Include all possible JTAG pins here. (Remember to use the GPIO listing and not pin number.)*/
unsigned char pins[] = {2, 3, 4, 17, 27, 22};

/* Give the pins pretty names. */
char * pinnames[] = { "GPIO2", "GPIO3", "GPIO4", "GPIO17",
                      "GPIO27", "GPIO22"};

/* GPIO Setup macros. */
#define INP_GPIO(g) (*(gpio+((g)/10)) &= ~(7<<(((g)%10)*3)))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))
#define GPIO_HI(g) *(gpio+7) = 1 << g
#define GPIO_LO(g) *(gpio+10) = 1 << g
#define GET_GPIO(g) (*(gpio+13)&(1<<g)) >> g // 0 if LOW, (1<<g) if HIGH


int jTDI, jTDO, jTMS, jTCK, jTRST;
int TDI , TDO , TCK , TMS ;

int port = 2345;
static char my_hostname[128];
char *string_url;
fd_set rfds;	/* read */
fd_set afds;	/* active */
int pSock;	/* passive socket */


int writeSocket(int, char *, int);
int readSocket(int, char *, int);
int push_buffer(int, char *, char *);
int do_help(int, char *, char *, char *);
int do_get(int, char *, char *, char *);
int do_null(int, char *, char *, char *);

#define MAX_BUFFER 8192
#define DEFAULT_QLEN 50
#define PUSH(x) push_buffer(s, buffer, x)
#define bzero(p, s) memset(p, 0, s);
#define ERR_CLOSED 5
#define USAGE " \
	[ -h ] This help \
	[ -p port ] (default : 2345) \
"
char buffer[MAX_BUFFER];

struct table_t {
	char *name;
	int (*function) (int, char *, char *, char *);
};

struct table_t tableCmds[] = {
	{ "HELP", do_help },
	{ "GET", do_get },
	{ "", do_null },
	{ "\r", do_null },
	{ NULL, do_null },
};

struct webcmd_t {
	char *name;
	unsigned int id;
};

struct webcmd_t webCmds[] = {
	{ "/", 1 },
	{ "/INFO", 5 },
	{ NULL, 0 },
};

unsigned int bit_reverse(unsigned int v) {
	unsigned int r = v;
	r = ((r >> 1)&0x55555555) | ((r & 0x55555555) << 1);
	r = ((r >> 2) & 0x33333333) | ((r & 0x33333333) << 2);
	r = ((r >> 4) & 0x0f0f0f0f) | ((r & 0x0f0f0f0f) << 4);
	r = ((r >> 8) & 0x00ff00ff) | ((r & 0x00ff00ff) << 8);
	r = ( r >> 16 ) | ( r << 16);
	return r;
}

void longfill(int val, int count) {
	jTDI = jTDO = jTMS = jTCK = jTRST = val;
}	

void set_all_high(int n) {
	int i = 0;
	for (i = 0; i < n; i++) {
		OUT_GPIO(pins[i]);
		GPIO_HI(pins[i]); 
	}
}

void display_progress(int n, int l) { }

void display_pins() {
	if (jTDI >= MAX_CHAN) 
		printf("TDI: N/A\n");
	else
		printf("TDI: %d\n", jTDI);
	printf("TDO: %d\n", jTDO);
	printf("TCK: %d\n", jTCK);
	printf("TMS: %d\n", jTMS);
}

void config(int tdi_pin, int tdo_pin, int tck_pin, int tms_pin) {
	TDI = tdi_pin; 
	TDO = tdo_pin; 
	TCK = tck_pin; 
	TMS = tms_pin;
	OUT_GPIO(pins[TDI]);
	OUT_GPIO(pins[TCK]);
	OUT_GPIO(pins[TMS]);
	INP_GPIO(pins[TDO]);
}

void TCK_Pulse() {
	GPIO_HI(pins[TCK]); 
	GPIO_LO(pins[TCK]); 
}

void TMS_Low() {
	GPIO_LO(pins[TMS]); 
	TCK_Pulse();
}

void TMS_High() {
	GPIO_HI(pins[TMS]); 
	TCK_Pulse();
}
void enter_shift_dr() {
	TMS_Low();
	TMS_High();
	TMS_Low();
	TMS_Low();
}

void enter_shift_ir() {
	TMS_Low();
	TMS_High();
	TMS_High();
	TMS_Low();
	TMS_Low();
}
void restore_idle() {
	int i;
	GPIO_HI(pins[TMS]); 
	for(i = 0; i < 4; i++) TCK_Pulse();
}

int bypass_test(int num, int bPattern) {
	int i, value;
	GPIO_LO(pins[TCK]);
	restore_idle();
	
	enter_shift_ir();
	GPIO_LO(pins[TMS]);
	GPIO_HI(pins[TDI]);
	for(i = 0; i < num << 6; i++) {
		TCK_Pulse();
	}
	TMS_High();
	TMS_High();
	TMS_High();
	TMS_Low();
	TMS_Low();

	for (i = 0; i < 32+num; i++) {
		value <<= 1;
		SET_GPIO_ALT(pins[TDI], bPattern & 1);
		GPIO_HI(pins[TCK]);
		value |= GET_GPIO(pins[TDO]);
		GPIO_LO(pins[TCK]);
		bPattern >>= 1;
	}
	TMS_High();
	TMS_High();
	TMS_Low();
	return(bit_reverse(value));
}





int get_device_id(int num) {
	int data = 0;
	GPIO_LO(pins[TCK]);
	restore_idle();
	enter_shift_dr();
	GPIO_LO(pins[TMS]);
	GPIO_HI(pins[TDI]);
	int i;
	for(i = 1; i < (num << 5); i++) {
		data <<=1;
		GPIO_HI(pins[TCK]);
		data |= GET_GPIO(pins[TDO]);
		GPIO_LO(pins[TCK]);
		if (i % 32 == 0)
			data = bit_reverse(data);
			printf("data: %08x\n", data);
			data = 0;
		}
	TMS_High();
	TMS_High();
	TMS_Low();
}

int detect_device() {
	int i, num;
	GPIO_LO(pins[TCK]);
	restore_idle();
	enter_shift_ir();
	GPIO_LO(pins[TMS]);
	GPIO_HI(pins[TDI]);
	for(i = 0; i < 1023; i++) TCK_Pulse();
	TMS_High();
	TMS_High();
	TMS_High();
	TMS_Low();
	TMS_Low();
	for(i = 0; i < 1024; i++) TCK_Pulse();

	GPIO_LO(pins[TDI]);
	for(num = 0; num < 1023; num++) {
		GPIO_HI(pins[TCK]);
		if (GET_GPIO(pins[TDO]) == 0) {
			GPIO_LO(pins[TCK]);
			return(num);
		}
		GPIO_LO(pins[TCK]);
	}
	if (num > 1023) 
		num = 0;
	TMS_High();
	TMS_Low();
	return(num);
}

void idcode_scan() {
	int value, new_value;
	jTDI = 0;
	for (jTDO=0; jTDO < 4; jTDO++) {
		for (jTCK = 0; jTCK < 4 ; jTCK++) {

			if (jTCK == jTDO) 
				continue;
			for (jTMS = 0; jTMS < 4 ; jTMS++) {
				if ((jTMS == jTCK) || (jTMS == jTDO))
					continue;
				
				set_all_high(4);
				config(jTDI, jTDO, jTCK, jTMS);
			
				value = get_device_id(1);
				if ((value != -1) && (value & 1)) {
					display_pins();

					for (jTRST = 0; jTRST < 4; jTRST++) {
						if ((jTRST == jTMS) || (jTRST == jTCK) || (jTRST == jTDO) || (jTRST == jTDI))
							continue;
						OUT_GPIO(pins[jTRST]);
						GPIO_LO(pins[jTRST]);
					
						new_value = get_device_id(1);
						if (new_value != value)	
							printf("TRST = %d\n", jTRST);
						GPIO_HI(pins[jTRST]);
					}
				}
			}
		}
	}
	longfill(0, 5);
	printf("IDCODE scan complete\n");
}

void bypass_scan() {
	int value, new_value, ctr=0;
	for(jTDI = 0; jTDI < 4; jTDI++) {
		for(jTDO = 0; jTDO < 4; jTDO++) {
			if (jTDO == jTDI)
				continue;
			for(jTCK = 0; jTCK < 4; jTCK++) {
				if ((jTCK == jTDO) || (jTCK == jTDI))
					continue;
				for(jTMS = 0; jTMS < 4; jTMS++) {
					if ((jTMS == jTCK) || (jTMS == jTDO) || (jTMS == jTDI))
						continue;
					
					set_all_high(4);
					config(jTDI, jTDO, jTCK, jTMS);
					value = detect_device();
					if (value) {
						display_pins();
						
						for(jTRST = 0; jTRST < 4; jTRST++) {
							if ((jTRST == jTMS) || (jTRST == jTCK) || (jTRST == jTDO) || (jTRST == jTDI))
								continue;
							OUT_GPIO(pins[jTRST]);
							GPIO_LO(pins[jTRST]);
							new_value = detect_device();
							if (new_value != value)
								printf("TRST = %d\n", jTRST);
							GPIO_HI(pins[jTRST]);
						}
						printf("number of devices detected: %d\n", value);
					}
					++ctr;
					display_progress(ctr, 10);
				}
			}
		}
	}
	longfill(0,5);
	printf("bypass complete\n");

}		

/* Check which version of RPI we are running on. */
void set_rpi_conf(){


	char buf[1024];
	unsigned long RPI_version = 0;
	char magic[]="boardrev=";
	int strsize = strlen(magic);
	int n = 0;
	int offset;
	int i;

	FILE *fp = fopen("/proc/cmdline", "r");
	if(!fp){
		printf("(ERROR) check_rpi_rev: File does not exist.\n");
		exit(-1);
	}

	n = fread(buf, 1, sizeof(buf)-1, fp);
	fclose(fp);

	for(i=0;i<n;i++){
		memcpy(magic, buf+i, strsize);
		magic[strsize]=0;
		if(!strncmp(magic, "boardrev=", strsize)){
			offset=i+strsize;
			i=i+strsize;
			while(buf[i]!=' '&&buf[i]!=0){
				i++;
			}

			strsize = i-offset;
			strncpy(buf, buf+offset, strsize);
			buf[strsize]=0;
			RPI_version = strtoul(buf, NULL, 0);
		}
	}


	switch(RPI_version){

		case 0x0002:
			printf("RaspberryPi 1 detected! (Model: B) (PCB Rev: 1.0) (Memory: 256MB)\n");
			PERI_BASE_ADDR = 0x20000000;
			//wiring = 0;
			break;

		case 0x0003:
			printf("RaspberryPi 1 detected! (Model: B(ECN001)) (PCB Rev: 1.0) (Memory: 256MB)\n");
			PERI_BASE_ADDR = 0x20000000;
			//wiring = 0;
			break;

		case 0x0004:
			printf("RaspberryPi 1 detected! (Model: B) (PCB Rev: 2.0) (Memory: 256MB)\n");
			PERI_BASE_ADDR = 0x20000000;
			//wiring = 1;
			break;

		case 0x0005:
			printf("RaspberryPi 1 detected! (Model: B) (PCB Rev: 2.0) (Memory: 256MB)\n");
			PERI_BASE_ADDR = 0x20000000;
			//wiring = 1;
			break;

		case 0x0006:
			printf("RaspberryPi 1 detected! (Model: B) (PCB Rev: 2.0) (Memory: 256MB)\n");
			PERI_BASE_ADDR = 0x20000000;
			//wiring = 1;
			break;

		case 0x0007:
			printf("RaspberryPi 1 detected! (Model: A) (PCB Rev: 2.0) (Memory: 256MB)\n");
			PERI_BASE_ADDR = 0x20000000;
			//wiring = 1;
			break;

		case 0x0008:
			printf("RaspberryPi 1 detected! (Model: A) (PCB Rev: 2.0) (Memory: 256MB)\n");
			PERI_BASE_ADDR = 0x20000000;
			//wiring = 1;
			break;

		case 0x0009:
			printf("RaspberryPi 1 detected! (Model: A) (PCB Rev: 2.0) (Memory: 256MB)\n");
			PERI_BASE_ADDR = 0x20000000;
			//wiring = 1;
			break;

		case 0x000D:
			printf("RaspberryPi 1 detected! (Model: B) (PCB Rev: 2.0) (Memory: 512MB)\n");
			PERI_BASE_ADDR = 0x20000000;
			//wiring = 1;
			break;

		case 0x000E:
			printf("RaspberryPi 1 detected! (Model: B) (PCB Rev: 2.0) (Memory: 512MB)\n");
			PERI_BASE_ADDR = 0x20000000;
			//wiring = 1;
			break;

		case 0x000F:
			printf("RaspberryPi 1 detected! (Model: B) (PCB Rev: 2.0) (Memory: 512MB)\n");
			PERI_BASE_ADDR = 0x20000000;
			//wiring = 1;
			break;

		case 0x0010:
			printf("RasbperryPi 1 detected! (Model: B+) (PCB Rev: 1.0) (Memory: 512MB)\n");
			PERI_BASE_ADDR = 0x20000000;
			//wiring = 2;
			break;

		case 0x0011:
			printf("RaspberryPi 1 detected! (Model: Computer Module) (PCB Rev: 1.0) (Memory: 512MB)\n");
			PERI_BASE_ADDR = 0x20000000;
			break;

		case 0x0012:
			printf("RaspberryPi 1 detected! (Model: A+) (PCB Rev: 1.0) (Memory: 256MB\n");
			PERI_BASE_ADDR = 0x20000000;
			//wiring = 2;
			break;
		case 0xa01041:
			printf("RaspberryPi 2 detected! (Model: B) (PCB Rev: 1.1, UK made) (Memory: 1GB)\n");
			PERI_BASE_ADDR = 0x3F000000;
			//wiring = 2;
			break;
		case 0xa21041:
			printf("RaspberryPi 2 detected! (Model: B) (PCB Rev: 1.1, CN made) (Memory: 1GB)\n");
			PERI_BASE_ADDR = 0x3F000000;
			//wiring = 2;
			break;
		default:
			printf("(ERROR) main: RaspberryPi board revision not supported! (0x%x)\n", RPI_version);
			exit(-1);
			break;
	}

 return;
}

void setup_io() {
	set_rpi_conf();
	
	if ((mem_fd = open("/dev/mem", O_RDWR | O_SYNC)) < 0) {
		fprintf(stderr, "(ERROR) setup_io: can't open /dev/mem (TRY RUNNING AS ROOT)\n");
		exit(-1);
	}
	gpio_map = mmap(
		NULL,
		4096,
		PROT_READ|PROT_WRITE,
		MAP_SHARED,
		mem_fd,
		GPIO_BASE
	);
	close(mem_fd);
	if (gpio_map == MAP_FAILED) {
		fprintf(stderr, "(ERROR) setup_io: mmap error %d\n", (int)gpio_map);
		exit(-1);
	}
	printf("Peripheral base: 0x%08x\n\n", PERI_BASE_ADDR);
	gpio = (volatile unsigned *)gpio_map;
	return;
}

int push_buffer(int s, char *buffer, char *add_buffer) {
	int l = 0, lp = 0;
	if (buffer) {
		l = strlen(buffer);
		if (add_buffer) {
			lp = strlen(add_buffer);
			if (l + lp > MAX_BUFFER) {
				writeSocket(s, buffer, l);
				bzero(buffer, MAX_BUFFER);	
			}
			sprintf(buffer, "%s%s", buffer, add_buffer);
		} else {
			writeSocket(s, buffer, strlen(buffer));
			bzero(buffer, MAX_BUFFER);
		}
	}
	return l+lp;
}

int do_null(int s, char *from, char *cmd, char *rest) {
	writeSocket(s, "> ", 2);
	return 0;
}
int do_help(int s, char *from, char *cmd, char *rest) {
	char buffer[MAX_BUFFER];
	int i;
	bzero(buffer, MAX_BUFFER);
	PUSH("HELP\r\n");
	return 0;
}

int do_get(int s, char *from, char *cmd, char *rest) {
	char *args;
	char *webcmd;
	char *decode = NULL;
	char *last_ptr = NULL;
	int l_args = 0, l_webcmd, l_decode, i, id_webcmd = 0;
	char hex[2];
	
	webcmd = (char *)strtok_r(rest, ":?", &last_ptr); l_webcmd = strlen(webcmd);
	args = (char *)strtok_r(NULL, ":?", &last_ptr);

	if (args) {
		l_args = strlen(args);
		if (l_args % 2 == 0) {
			l_decode = MAX(strlen(my_hostname), l_args / 2);
			decode = (char *)malloc(sizeof(char) * l_decode);
			bzero(decode, l_decode);
			for(i = 0; i < l_decode; i++) {
				hex[0] = args[2*i]; hex[1] = args[2*i+1];
				decode[i] = strtol(hex, (char **)NULL, 16);
			}
			l_decode = strlen(decode);
			free(decode);
		} else {
			fprintf(stderr, "unpaired arg\n");
		}
	}

	for(i = 0; webCmds[i].name; i++) {
		if (!strcasecmp(webcmd, webCmds[i].name)) {
			id_webcmd = webCmds[i].id;
			break;
		}
	}
	switch (id_webcmd) {
		case 0:	//NULL
			break;
		case 1: 
			break;
		default:
			break;
	}
	return 1;
}

int readSocket(int s, char *buffer, int size) {
	return(read(s, buffer, size));
}
int writeSocket(int s, char *buffer, int size) {
	return write(s, buffer, size);
}
int passiveSocket(int port, int type, int qlen) {
	struct hostent *ipServer;
	struct sockaddr_in serverAddr;
	int s;
	int one = 1;
	
	if (( s = socket(AF_INET, type, 0)) == -1) {
		fprintf(stderr, "Can't create socket\n");
		exit(1);
	}
	if (setsockopt(s, SOL_SOCKET, SO_REUSEADDR, (char *)&one, sizeof(int)) < 0) {
		perror("setsockopt");
		exit(1);
	}
	gethostname(my_hostname, 128);
	if (!(ipServer = gethostbyname(my_hostname))) {
		fprintf(stderr, "no existence!\n");
		exit(1);
	}
	bzero((char *)&serverAddr, sizeof(struct sockaddr_in));
	serverAddr.sin_family = AF_INET;
	serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	serverAddr.sin_port = htons(port);
	
	string_url = (char *)malloc(sizeof(char) * (strlen(my_hostname) + 6));
	sprintf(string_url, "%s:%d", my_hostname, port);
	
	if ((bind(s, (struct sockaddr *)&serverAddr, sizeof(struct sockaddr_in))) < 0) {
		close(s);
		perror("bind");
		exit(1);
	}
	if (type == SOCK_STREAM && listen(s, qlen)) {
		fprintf(stderr, "Can't listen on claimed port\n");
		exit(1);
	}
	return s;
}
int acceptConnection(int s, struct sockaddr_in *clientAddr) {
	int alen = sizeof(struct sockaddr_in);
	int r,e;
		r = accept(s, (struct sockaddr *)clientAddr, &alen);
		if (r == 0) {
			return 0;
		}
		if (r == -1 && errno != EINTR) {
			e = errno;
			perror("accept");
			fprintf(stderr, "Can't accept connection (%d)\n", e);
		}
	return r;
}

int parseLine(int s, char *buf) {
	char *buffer = buf;
	char *cmd = NULL;
	char *rest = NULL;
	const char delimiters[] = " \t\r";
	int i;
	char *last_ptr = NULL;
	
	cmd = (char *)strtok_r(buffer, delimiters, &last_ptr);
	rest = (char *)strtok_r(NULL, delimiters, &last_ptr);
	fprintf(stderr, "**%s**%s**\n", cmd?cmd:"NULL", rest?rest:"NULL");
	if (cmd) {
		for(i = 0; tableCmds[i].name ; i++) {
			if (!strcasecmp(cmd, tableCmds[i].name)) {
				break;
			}
		}
		return tableCmds[i].function(s, (void*)0, cmd, rest);
	} else 
		return do_null(s, (void *)0, cmd, rest);
}

int run(int s, int welcome) {
	char buffer[MAX_BUFFER];
	char header[40];
	bzero(buffer, MAX_BUFFER);
	
	if (!readSocket(s, buffer, MAX_BUFFER)) {
		return ERR_CLOSED;
	}

	fprintf(stderr, "[%s] read\n", buffer);
	if (buffer) {
		return parseLine(s, buffer);
	} else {
		printf("bad, empty client buffer\n");
		return ERR_CLOSED;
	}
}

int main(int argc, char **argv) {
	setup_io();
	

	int qlen = DEFAULT_QLEN;
	int nfds, errflag = 0;
	struct sockaddr_in clientAddr;
	
	int c;
	while (( c = getopt(argc, argv, "p:")) != EOF) {
		switch(c) {
		case 'p':
			port = atoi(optarg);
			printf("starting server on port %d\n", port);
			break;
		default:
			errflag = 1;	
			break;
		}
	}

	if (errflag) {
		fprintf(stderr, "Usage: %s %s\n", *argv, USAGE);
		exit(EXIT_FAILURE);
	}
	pSock = passiveSocket(port, SOCK_STREAM, qlen);
	fcntl(pSock, F_SETFL, O_NDELAY);
	nfds = getdtablesize();
	FD_ZERO(&afds);
	FD_SET(pSock, &afds);

	while (1) {
		int fd;
		bcopy((char *)&afds, (char *)&rfds, sizeof(rfds));
		if (select(nfds, &rfds, (fd_set *)0, (fd_set *)0, (struct timeval *)0) < 0 ) {
			perror("select");
		}
		if (FD_ISSET(pSock, &rfds)) {
			int s = acceptConnection(pSock, &clientAddr);
			struct in_addr address = clientAddr.sin_addr;
			printf("new client: %s\n", inet_ntoa(address));
			FD_SET(s, &afds);
			bzero(buffer, MAX_BUFFER);
			sprintf(buffer, "<!-- Welcome ... (type HELP for help)\n\n-->");
			writeSocket(s, buffer, strlen(buffer));
		}
		
		for(fd = 0; fd < nfds; ++fd) {
			if (fd != pSock && FD_ISSET(fd, &rfds)) {
				if (run(fd, 0)) {
					close(fd);
					printf("Client lost\n");
					FD_CLR(fd, &afds);
				}
			}
		}
	}
	close(pSock);
	printf("End of process\n");
	return(0);
	
				
}

	
			

